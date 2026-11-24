#!/usr/bin/env python3
"""Stage an ODrive firmware image over CAN and request bootloader installation.

The application first stops its axes, then receives the image into the Flash
buffer. The bootloader verifies and installs the image after the application
restarts. Windows defaults to PCAN; other platforms default to SocketCAN.
"""

import argparse
import queue
import struct
import sys
import threading
import time
import zlib


CAN_ID = 0xABC
CHUNK_SIZE = 1024
MAX_FIRMWARE_SIZE = 320 * 1024
FRAME_START = 0x01
FRAME_DATA = 0x02
FRAME_END = 0x03
FRAME_UPGRADE_REQUEST = 0x10
FRAME_STATUS_REQUEST = 0x11
FRAME_METADATA = 0x12
FRAME_ACK = 0x80
IMAGE_TYPES = {"app": 1, "parameters": 2}

STATUS_NAMES = {
    0: "ready",
    1: "accepted",
    2: "checksum mismatch",
    3: "invalid or missing data frame",
    4: "unexpected packet sequence",
    5: "invalid firmware size",
    6: "Flash operation failed",
    7: "transfer complete",
    8: "firmware is already installed",
    9: "device is stopping its axes",
    10: "device is stopped",
    11: "metadata fragment accepted",
    12: "full image checksum mismatch",
    13: "device is busy",
    15: "device requests exact byte size",
}


def make_message(can, payload):
    return can.Message(
        arbitration_id=CAN_ID,
        is_extended_id=True,
        data=payload,
    )


def send_control(bus, can, payload, log):
    bus.send(make_message(can, payload))
    if log:
        log("TX 0xABC EXT [{}] {}".format(len(payload), payload.hex(" ")))


def wait_for_ack(bus, can, sequence, timeout, log=None):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            if log:
                log("ACK timeout: seq={}".format(sequence))
            raise TimeoutError("timed out waiting for device acknowledgement")
        message = bus.recv(remaining)
        if message is None or message.is_error_frame or message.is_remote_frame:
            continue
        if (message.arbitration_id != CAN_ID or not message.is_extended_id or
                len(message.data) != 8 or message.data[0] != FRAME_ACK):
            continue
        ack_sequence = struct.unpack_from("<H", message.data, 2)[0]
        if ack_sequence == sequence:
            status = message.data[1]
            expected = struct.unpack_from("<H", message.data, 4)[0]
            if log:
                log("RX 0xABC EXT [8] {} | {} seq={} expected={}".format(
                    message.data.hex(" "), STATUS_NAMES.get(status, status), sequence, expected))
            return status, expected


def send_chunk(bus, can, sequence, image_size, chunk, args, log_callback=None):
    checksum = zlib.crc32(chunk) & 0xFFFFFFFF

    for attempt in range(args.retries + 1):
        start = bytes([FRAME_START, 0]) + struct.pack(
            "<HI", sequence, image_size
        )
        if log_callback:
            log_callback("chunk={} CRC32=0x{:08X} attempt={}/{}".format(
                sequence, checksum, attempt + 1, args.retries + 1))
        send_control(bus, can, start, log_callback)

        try:
            status, expected = wait_for_ack(bus, can, sequence, args.timeout, log_callback)
        except TimeoutError:
            if attempt == args.retries:
                raise
            continue
        if status != 0:
            if status in (2, 3) and attempt < args.retries:
                continue
            raise RuntimeError(
                "device rejected chunk {}: {} (expected sequence {})".format(
                    sequence, STATUS_NAMES.get(status, "status {}".format(status)),
                    expected
                )
            )

        for frame_index, offset in enumerate(range(0, CHUNK_SIZE, 6)):
            payload = chunk[offset:offset + 6].ljust(6, b"\xFF")
            bus.send(make_message(can, bytes([FRAME_DATA, frame_index]) + payload))
            if args.frame_delay:
                time.sleep(args.frame_delay)

        end = bytes([FRAME_END]) + struct.pack("<HI", sequence, checksum)
        if log_callback:
            log_callback("TX DATA 0x02: index=0..{}, 6 bytes/frame".format(frame_index))
        send_control(bus, can, end, log_callback)

        try:
            status, expected = wait_for_ack(bus, can, sequence, args.timeout, log_callback)
        except TimeoutError:
            if attempt == args.retries:
                raise
            continue

        if status == (7 if (sequence + 1) * CHUNK_SIZE >= image_size else 1):
            return status
        if status in (2, 3) and attempt < args.retries:
            message = "Chunk {} rejected ({}); retrying".format(
                sequence, STATUS_NAMES[status])
            if log_callback:
                log_callback(message)
            else:
                print(message, file=sys.stderr)
            continue
        raise RuntimeError(
            "device rejected chunk {}: {} (expected sequence {})".format(
                sequence, STATUS_NAMES.get(status, "status {}".format(status)),
                expected
            )
        )

    raise RuntimeError("chunk {} failed after retries".format(sequence))


def send_and_wait(bus, can, payload, sequence, timeout, retries, log=None):
    for attempt in range(retries + 1):
        send_control(bus, can, payload, log)
        try:
            return wait_for_ack(bus, can, sequence, timeout, log)
        except TimeoutError:
            if attempt == retries:
                raise


def read_image(firmware_path, image_type):
    with open(firmware_path, "rb") as firmware_file:
        firmware = firmware_file.read(MAX_FIRMWARE_SIZE + 1)
    if not firmware:
        raise ValueError("固件文件为空")
    if image_type not in IMAGE_TYPES:
        raise ValueError("请选择应用程序或参数")
    if image_type == "app":
        if not 200 * 1024 <= len(firmware) <= MAX_FIRMWARE_SIZE:
            raise ValueError("应用镜像必须为 200–320 KiB")
        sp, entry = struct.unpack_from("<II", firmware)
        if not (0x20000000 < sp <= 0x20020000 and sp % 8 == 0 and entry & 1
                and 0x08010000 <= (entry & ~1) < 0x08010000 + len(firmware)):
            raise ValueError("应用向量表无效，请使用为 0x08010000 链接的应用 .bin")
    elif len(firmware) > 128 * 1024:
        raise ValueError("参数扇区镜像不能超过 128 KiB")
    return firmware


def image_parameters(firmware):
    size = len(firmware)
    return size, zlib.crc32(firmware) & 0xFFFFFFFF, (size + CHUNK_SIZE - 1) // CHUNK_SIZE


def transfer_firmware(firmware_path, interface, channel, bitrate,
                      image_type="app", timeout=5.0,
                      retries=5, frame_delay=0.001, can_module=None,
                      progress_callback=None, log_callback=None, *,
                      image_size=None, image_crc=None, chunk_count=None):
    firmware = read_image(firmware_path, image_type)
    actual = image_parameters(firmware)
    for label, supplied, calculated in zip(
            ("固件大小", "整包 CRC-32", "分片数"), (image_size, image_crc, chunk_count), actual):
        if supplied is not None and supplied != calculated:
            raise ValueError("{}与所选文件不符：输入 {}，计算值 {}".format(label, supplied, calculated))
    image_size, image_crc, chunk_count = actual
    if log_callback:
        log_callback("{} {} {} bit/s | type={} size={} CRC32=0x{:08X} chunks={}".format(
            interface, channel, bitrate, image_type, image_size, image_crc, chunk_count))

    if can_module is None:
        try:
            import can as can_module
        except ImportError as error:
            raise RuntimeError(
                "缺少 python-can，请运行：python -m pip install python-can"
            ) from error

    transfer_args = argparse.Namespace(
        timeout=timeout,
        retries=retries,
        frame_delay=frame_delay,
    )
    with can_module.Bus(interface=interface, channel=channel, bitrate=bitrate) as bus:
        type_id = IMAGE_TYPES[image_type]
        request = struct.pack("<BBHI", FRAME_UPGRADE_REQUEST, type_id, chunk_count, image_crc)
        status, _ = send_and_wait(
            bus, can_module, request, 0, timeout, retries, log_callback
        )
        if status != 15:
            raise RuntimeError("设备拒绝类型请求或协议不兼容：{}".format(STATUS_NAMES.get(status, status)))
        status, _ = send_and_wait(bus, can_module, struct.pack(
            "<BBHI", FRAME_METADATA, type_id, chunk_count, image_size
        ), 0, timeout, retries, log_callback)
        if status == 8:
            return image_size, chunk_count, True
        if status not in (9, 10):
            raise RuntimeError(
                "设备拒绝升级请求：{}".format(
                    STATUS_NAMES.get(status, "status {}".format(status))
                )
            )

        stop_deadline = time.monotonic() + 120.0
        while status != 10:
            if time.monotonic() >= stop_deadline:
                raise TimeoutError("等待设备停机超时")
            if status == 9:
                time.sleep(0.1)
            status, _ = send_and_wait(
                bus, can_module, bytes([FRAME_STATUS_REQUEST]), 0, timeout, retries, log_callback
            )
            if status not in (9, 10):
                raise RuntimeError(
                    "设备停止状态异常：{}".format(
                        STATUS_NAMES.get(status, "status {}".format(status))
                    )
                )

        for sequence in range(chunk_count):
            offset = sequence * CHUNK_SIZE
            chunk = firmware[offset:offset + CHUNK_SIZE].ljust(CHUNK_SIZE, b"\xFF")
            status = send_chunk(
                bus, can_module, sequence, len(firmware), chunk, transfer_args,
                log_callback=log_callback,
            )
            if progress_callback:
                progress_callback(sequence + 1, chunk_count, status)
    return len(firmware), chunk_count, False


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Send a firmware image to the ODrive Flash buffer over CAN"
    )
    parser.add_argument("firmware", help="firmware binary file")
    default_interface = "pcan" if sys.platform == "win32" else "socketcan"
    parser.add_argument("--interface", default=default_interface,
                        help="python-can interface (default: {})".format(default_interface))
    parser.add_argument("--channel", required=True,
                        help="CAN channel, for example can0 or PCAN_USBBUS1")
    parser.add_argument("--type", choices=IMAGE_TYPES, default="app",
                        help="target image: app (default) or parameters")
    for option in ("size", "crc32", "chunks"):
        parser.add_argument("--" + option, type=lambda value: int(value, 0),
                            help="expected image {}; calculated if omitted".format(option))
    parser.add_argument("--bitrate", type=int, default=100000,
                        help="CAN bitrate in bit/s (default: 100000)")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="acknowledgement timeout in seconds (default: 5)")
    parser.add_argument("--retries", type=int, default=5,
                        help="maximum retries per 1 KiB chunk (default: 5)")
    parser.add_argument("--frame-delay", type=float, default=0.001,
                        help="delay between data frames in seconds (default: 0.001)")
    args = parser.parse_args(argv)

    if args.timeout <= 0 or args.retries < 0 or args.frame_delay < 0:
        parser.error("timeout must be positive and retries/frame-delay non-negative")

    try:
        firmware_size, chunk_count, already_current = transfer_firmware(
            args.firmware, args.interface, args.channel, args.bitrate,
            image_type=args.type,
            image_size=args.size, image_crc=args.crc32, chunk_count=args.chunks,
            log_callback=print,
            timeout=args.timeout,
            retries=args.retries,
            frame_delay=args.frame_delay,
            progress_callback=lambda current, total, status: print(
                "Chunk {}/{} {}".format(
                    current, total, "complete" if status == 7 else "saved"
                ),
                flush=True,
            ),
        )
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(str(error))

    if already_current:
        print("Device already has this firmware.")
    else:
        print("Firmware staged: {} bytes, {} chunks. Device is rebooting "
              "to install it.".format(firmware_size, chunk_count))
    return 0


def launch_gui():
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
        from tkinter.scrolledtext import ScrolledText
    except ImportError as error:
        print(
            "Tkinter is not installed. Install the Python Tk package "
            "(for Ubuntu/Debian: sudo apt install python3-tk).\n{}".format(error),
            file=sys.stderr,
        )
        return 2

    default_interface = "pcan" if sys.platform == "win32" else "socketcan"
    common_interfaces = [
        "pcan", "socketcan", "slcan", "vector", "kvaser", "ixxat",
        "canalystii", "usb2can", "seeedstudio", "virtual",
    ]
    try:
        import can
        interfaces = sorted(set(common_interfaces) | set(can.interface.BACKENDS))
    except ImportError:
        can = None
        interfaces = common_interfaces
    if default_interface not in interfaces:
        interfaces.append(default_interface)

    root = tk.Tk()
    root.title("ODrive CAN 固件下载工具")
    root.geometry("760x660")
    root.minsize(650, 540)
    events = queue.Queue()

    firmware_path = tk.StringVar()
    interface_name = tk.StringVar(value=default_interface)
    channel_name = tk.StringVar(
        value="PCAN_USBBUS1" if default_interface == "pcan" else ""
    )
    bitrate_value = tk.StringVar(value="100000")
    status_text = tk.StringVar(value="请选择固件并确认 CAN 参数。")

    main_frame = ttk.Frame(root, padding=14)
    main_frame.pack(fill="both", expand=True)
    main_frame.columnconfigure(1, weight=1)

    ttk.Label(main_frame, text="固件文件 (.bin)：").grid(
        row=0, column=0, sticky="w", padx=(0, 8), pady=6
    )
    firmware_entry = ttk.Entry(main_frame, textvariable=firmware_path)
    firmware_entry.grid(row=0, column=1, sticky="ew", pady=6)

    def choose_firmware():
        path = filedialog.askopenfilename(
            title="选择固件文件",
            filetypes=(("固件二进制文件", "*.bin"), ("所有文件", "*.*")),
        )
        if path:
            firmware_path.set(path)

    ttk.Button(main_frame, text="浏览...", command=choose_firmware).grid(
        row=0, column=2, padx=(8, 0), pady=6
    )

    ttk.Label(main_frame, text="CAN 接口：").grid(
        row=1, column=0, sticky="w", padx=(0, 8), pady=6
    )
    interface_combo = ttk.Combobox(
        main_frame, textvariable=interface_name, values=interfaces, state="readonly"
    )
    interface_combo.grid(row=1, column=1, sticky="ew", pady=6)

    ttk.Label(main_frame, text="CAN 端口/通道：").grid(
        row=2, column=0, sticky="w", padx=(0, 8), pady=6
    )
    channel_combo = ttk.Combobox(main_frame, textvariable=channel_name, state="normal")
    channel_combo.grid(row=2, column=1, sticky="ew", pady=6)
    refresh_button = ttk.Button(main_frame, text="扫描端口")
    refresh_button.grid(row=2, column=2, padx=(8, 0), pady=6)

    ttk.Label(main_frame, text="CAN 波特率：").grid(
        row=3, column=0, sticky="w", padx=(0, 8), pady=6
    )
    bitrate_combo = ttk.Combobox(
        main_frame,
        textvariable=bitrate_value,
        values=("100000", "125000", "250000", "500000", "800000", "1000000"),
        state="readonly",
    )
    bitrate_combo.grid(row=3, column=1, sticky="w", pady=6)
    ttk.Label(main_frame, text="bit/s").grid(
        row=3, column=2, sticky="w", padx=(8, 0), pady=6
    )

    params = ttk.LabelFrame(main_frame, text="固件参数（十进制或 0x 十六进制）", padding=6)
    params.grid(row=4, column=0, columnspan=3, sticky="ew")
    fields = {}
    type_var = tk.StringVar(value="app")
    ttk.Label(params, text="镜像类型").grid(row=0, column=0, sticky="w")
    ttk.Combobox(params, textvariable=type_var, values=tuple(IMAGE_TYPES),
                 state="readonly").grid(row=0, column=1)
    for i, (key, label) in enumerate((
        ("image_size", "大小（字节）"),
        ("image_crc", "整包 CRC-32"), ("chunk_count", "分片数（1 KiB/片）"),
    )):
        variable = tk.StringVar()
        fields[key] = (label, variable)
        row, col = divmod(i, 2)
        row += 1
        ttk.Label(params, text=label).grid(row=row, column=col * 2, sticky="w")
        ttk.Entry(params, textvariable=variable, width=16).grid(
            row=row, column=col * 2 + 1, padx=6, pady=2)

    def calculate():
        try:
            size, crc, chunks = image_parameters(read_image(firmware_path.get().strip(), type_var.get()))
        except (ValueError, OSError) as error:
            messagebox.showerror("无法计算固件参数", str(error), parent=root)
            return
        for key, value in (("image_size", size), ("image_crc", "0x{:08X}".format(crc)),
                           ("chunk_count", chunks)):
            fields[key][1].set(value)
        status_text.set("已计算大小、CRC-32 和分片数；请确认目标分区。")

    ttk.Button(params, text="一键计算", command=calculate).grid(row=1, column=4, rowspan=2)

    progress = ttk.Progressbar(main_frame, mode="determinate")
    progress.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(14, 4))
    status_label = ttk.Label(main_frame, textvariable=status_text, wraplength=680)
    status_label.grid(row=6, column=0, columnspan=3, sticky="w", pady=4)

    start_button = ttk.Button(main_frame, text="开始下载")
    start_button.grid(row=7, column=0, columnspan=3, pady=10)

    instructions = (
        "使用说明\n"
        "1. 选择类型及 .bin：app 为 200–320 KiB；parameters 为 NVM 参数镜像，最多 128 KiB。\n"
        "2. 选择 CAN 接口并扫描端口，也可以手动输入通道名；选择与设备一致的波特率。\n"
        "3. 点击“开始下载”。应用层会先请求停机，接收并校验固件后自动重启安装。CAN ID 为扩展帧 0xABC。\n\n"
        "Python 支持包与驱动\n"
        "• 必装：python -m pip install python-can\n"
        "• PCAN-USB：安装 PEAK PCAN 驱动/PCAN-Basic；接口选 pcan，端口通常为 PCAN_USBBUS1。\n"
        "• SLCAN 串口设备：另需 python -m pip install pyserial；端口通常为 COMx。\n"
        "• SocketCAN：仅 Linux；先配置好系统 CAN 接口。\n"
        "• Tkinter 通常随 Windows Python 提供；Linux 若缺少，请安装 python3-tk。"
    )
    tabs = ttk.Notebook(main_frame)
    tabs.grid(row=8, column=0, columnspan=3, sticky="nsew")
    main_frame.rowconfigure(8, weight=1)
    log_tab = ttk.Frame(tabs)
    tabs.add(log_tab, text="更新日志")
    tabs.add(ttk.Label(tabs, text=instructions, justify="left", wraplength=620),
             text="使用说明")
    history = ScrolledText(log_tab, height=8, width=60, state="disabled")
    history.pack(fill="both", expand=True)

    def append_log(line):
        history.configure(state="normal")
        history.insert("end", "{}  {}\n".format(time.strftime("%H:%M:%S"), line))
        history.see("end")
        history.configure(state="disabled")

    worker_running = {"transfer": False, "detection": False}

    def refresh_channels():
        if worker_running["detection"]:
            return
        selected_interface = interface_name.get()
        worker_running["detection"] = True
        refresh_button.configure(state="disabled")
        status_text.set("正在扫描 CAN 端口...")

        def detect():
            try:
                import can as can_module
            except ImportError as error:
                events.put(("detection_error",
                            "缺少 python-can：python -m pip install python-can"))
                return
            try:
                configs = can_module.detect_available_configs(
                    interfaces=[selected_interface], timeout=5.0
                )
                channels = []
                for config in configs:
                    channel = config.get("channel")
                    if channel is not None and channel not in channels:
                        channels.append(channel)
                events.put(("channels", channels))
            except (can_module.CanError, OSError, ValueError) as error:
                events.put(("detection_error", str(error)))

        threading.Thread(target=detect, daemon=True).start()

    refresh_button.configure(command=refresh_channels)
    interface_combo.bind("<<ComboboxSelected>>",
                         lambda _event: refresh_channels())

    def start_transfer():
        if worker_running["transfer"]:
            return
        path = firmware_path.get().strip()
        if not path:
            messagebox.showerror("缺少固件", "请先选择 .bin 固件文件。", parent=root)
            return
        if not channel_name.get().strip():
            messagebox.showerror(
                "缺少 CAN 端口", "请选择扫描到的 CAN 端口，或手动输入通道名。",
                parent=root,
            )
            return
        try:
            bitrate = int(bitrate_value.get())
        except ValueError:
            messagebox.showerror("波特率无效", "请输入有效的 CAN 波特率。", parent=root)
            return
        entered = {"image_type": type_var.get()}
        for key, (label, variable) in fields.items():
            try:
                value = int(variable.get().strip(), 0)
                if not 0 <= value <= 0xFFFFFFFF:
                    raise ValueError()
            except ValueError:
                messagebox.showerror("参数无效", "{}：请输入 32 位无符号整数。".format(label),
                                     parent=root)
                return
            entered[key] = value

        config = {
            "path": path,
            "interface": interface_name.get(),
            "channel": channel_name.get().strip(),
            "bitrate": bitrate,
        }
        worker_running["transfer"] = True
        start_button.configure(state="disabled")
        progress.configure(maximum=1, value=0)
        status_text.set("正在连接 CAN 设备...")

        def transfer():
            try:
                import can as can_module
                result = transfer_firmware(
                    config["path"],
                    config["interface"],
                    config["channel"],
                    config["bitrate"],
                    **entered,
                    can_module=can_module,
                    progress_callback=lambda current, total, _status: events.put(
                        ("progress", current, total)
                    ),
                    log_callback=lambda message: events.put(("log", message)),
                )
            except ImportError as error:
                events.put(("transfer_error",
                            "缺少 python-can：python -m pip install python-can\n{}".format(error)))
            except (can_module.CanError, OSError, ValueError, RuntimeError) as error:
                events.put(("transfer_error", str(error)))
            else:
                events.put(("done", result))

        threading.Thread(target=transfer, daemon=True).start()

    start_button.configure(command=start_transfer)

    def process_events():
        try:
            while True:
                event = events.get_nowait()
                kind = event[0]
                if kind == "channels":
                    channels = event[1]
                    channel_combo.configure(values=channels)
                    if channels and not channel_name.get():
                        channel_name.set(channels[0])
                    status_text.set(
                        "扫描到 {} 个端口。".format(len(channels))
                        if channels else "未自动发现端口；可手动输入通道名。"
                    )
                    worker_running["detection"] = False
                    refresh_button.configure(state="normal")
                elif kind == "detection_error":
                    status_text.set("端口扫描失败：{}".format(event[1]))
                    worker_running["detection"] = False
                    refresh_button.configure(state="normal")
                elif kind == "progress":
                    current, total = event[1], event[2]
                    progress.configure(maximum=total, value=current)
                    status_text.set("已保存分片 {}/{}。".format(current, total))
                elif kind == "log":
                    append_log(event[1])
                elif kind == "transfer_error":
                    worker_running["transfer"] = False
                    start_button.configure(state="normal")
                    status_text.set("下载失败：{}".format(event[1]))
                    append_log(status_text.get())
                    messagebox.showerror("下载失败", event[1], parent=root)
                elif kind == "done":
                    size, chunks, already_current = event[1]
                    worker_running["transfer"] = False
                    start_button.configure(state="normal")
                    if already_current:
                        status_text.set("设备已安装该固件，无需升级。")
                        messagebox.showinfo("无需升级", status_text.get(), parent=root)
                    else:
                        status_text.set(
                            "固件已校验并写入缓冲区，设备正在重启安装。"
                        )
                        append_log(status_text.get())
                        messagebox.showinfo(
                            "下载完成",
                            "固件已写入设备 Flash 缓冲区，设备正在重启安装。\n"
                            "大小：{} 字节\n分片：{}".format(size, chunks),
                            parent=root,
                        )
        except queue.Empty:
            pass
        root.after(100, process_events)

    def close_window():
        if worker_running["transfer"]:
            messagebox.showwarning(
                "正在下载", "请等待固件下载完成后再关闭窗口。", parent=root
            )
            return
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close_window)
    root.after(100, process_events)
    refresh_channels()
    root.mainloop()
    return 0


if __name__ == "__main__":
    if len(sys.argv) == 1 or sys.argv[1:] == ["--gui"]:
        sys.exit(launch_gui())
    sys.exit(main())
