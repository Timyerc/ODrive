#!/usr/bin/env python3
"""Transfer a firmware image to the ODrive application Flash buffer over CAN.

CAN ID 0xABC is an extended ID. A transfer uses an 8-byte start frame, indexed
6-byte data frames, and an end frame carrying the CRC-32 of the padded 1 KiB
chunk. The device acknowledges each chunk and asks the sender to retry on error.
Windows defaults to the PCAN interface; other platforms default to SocketCAN.
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
MAX_BUFFER_SIZE = 3 * 128 * 1024
FRAME_START = 0x01
FRAME_DATA = 0x02
FRAME_END = 0x03
FRAME_ACK = 0x80

STATUS_NAMES = {
    0: "ready",
    1: "accepted",
    2: "checksum mismatch",
    3: "invalid or missing data frame",
    4: "unexpected packet sequence",
    5: "invalid firmware size",
    6: "Flash operation failed",
    7: "transfer complete",
}


def make_message(can, payload):
    return can.Message(
        arbitration_id=CAN_ID,
        is_extended_id=True,
        data=payload,
    )


def wait_for_ack(bus, can, sequence, timeout):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("timed out waiting for device acknowledgement")
        message = bus.recv(remaining)
        if message is None or message.is_error_frame or message.is_remote_frame:
            continue
        if (message.arbitration_id != CAN_ID or not message.is_extended_id or
                len(message.data) != 8 or message.data[0] != FRAME_ACK):
            continue
        ack_sequence = struct.unpack_from("<H", message.data, 2)[0]
        if ack_sequence == sequence:
            return message.data[1], struct.unpack_from("<H", message.data, 4)[0]


def send_chunk(bus, can, sequence, image_size, chunk, args, log_callback=None):
    checksum = zlib.crc32(chunk) & 0xFFFFFFFF

    for attempt in range(args.retries + 1):
        reset_flag = 1 if sequence == 0 and attempt == 0 else 0
        start = bytes([FRAME_START, reset_flag]) + struct.pack(
            "<HI", sequence, image_size
        )
        bus.send(make_message(can, start))

        try:
            status, expected = wait_for_ack(bus, can, sequence, args.timeout)
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
        bus.send(make_message(can, end))

        try:
            status, expected = wait_for_ack(bus, can, sequence, args.timeout)
        except TimeoutError:
            if attempt == args.retries:
                raise
            continue

        if status in (1, 7):
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


def transfer_firmware(firmware_path, interface, channel, bitrate, timeout=5.0,
                      retries=5, frame_delay=0.001, can_module=None,
                      progress_callback=None, log_callback=None):
    try:
        with open(firmware_path, "rb") as firmware_file:
            firmware = firmware_file.read()
    except OSError:
        raise

    if not firmware:
        raise ValueError("固件文件为空")
    if len(firmware) > MAX_BUFFER_SIZE:
        raise ValueError(
            "固件过大：{} 字节，缓冲区上限为 {} 字节".format(
                len(firmware), MAX_BUFFER_SIZE
            )
        )

    if can_module is None:
        try:
            import can as can_module
        except ImportError as error:
            raise RuntimeError(
                "缺少 python-can，请运行：python -m pip install python-can"
            ) from error

    chunk_count = (len(firmware) + CHUNK_SIZE - 1) // CHUNK_SIZE
    transfer_args = argparse.Namespace(
        timeout=timeout,
        retries=retries,
        frame_delay=frame_delay,
    )
    with can_module.Bus(interface=interface, channel=channel, bitrate=bitrate) as bus:
        for sequence in range(chunk_count):
            offset = sequence * CHUNK_SIZE
            chunk = firmware[offset:offset + CHUNK_SIZE].ljust(CHUNK_SIZE, b"\xFF")
            status = send_chunk(
                bus, can_module, sequence, len(firmware), chunk, transfer_args,
                log_callback=log_callback,
            )
            if progress_callback:
                progress_callback(sequence + 1, chunk_count, status)
    return len(firmware), chunk_count


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
        firmware_size, chunk_count = transfer_firmware(
            args.firmware, args.interface, args.channel, args.bitrate,
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

    print("Firmware transfer complete: {} bytes, {} chunks".format(
        firmware_size, chunk_count
    ))
    return 0


def launch_gui():
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
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
    root.geometry("720x580")
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

    progress = ttk.Progressbar(main_frame, mode="determinate")
    progress.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(14, 4))
    status_label = ttk.Label(main_frame, textvariable=status_text, wraplength=680)
    status_label.grid(row=5, column=0, columnspan=3, sticky="w", pady=4)

    start_button = ttk.Button(main_frame, text="开始下载")
    start_button.grid(row=6, column=0, columnspan=3, pady=10)

    instructions = (
        "使用说明\n"
        "1. 选择 .bin 固件文件；确认固件大小不超过 384 KiB。\n"
        "2. 选择 CAN 接口并扫描端口，也可以手动输入通道名；选择与设备一致的波特率。\n"
        "3. 点击“开始下载”，等待所有分片校验并保存完成。CAN ID 固定为扩展帧 0xABC。\n\n"
        "Python 支持包与驱动\n"
        "• 必装：python -m pip install python-can\n"
        "• PCAN-USB：安装 PEAK PCAN 驱动/PCAN-Basic；接口选 pcan，端口通常为 PCAN_USBBUS1。\n"
        "• SLCAN 串口设备：另需 python -m pip install pyserial；端口通常为 COMx。\n"
        "• SocketCAN：仅 Linux；先配置好系统 CAN 接口。\n"
        "• Tkinter 通常随 Windows Python 提供；Linux 若缺少，请安装 python3-tk。"
    )
    help_label = ttk.Label(
        main_frame, text=instructions, justify="left", wraplength=680
    )
    help_label.grid(row=7, column=0, columnspan=3, sticky="w", pady=(10, 0))

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
                    status_text.set(event[1])
                elif kind == "transfer_error":
                    worker_running["transfer"] = False
                    start_button.configure(state="normal")
                    status_text.set("下载失败：{}".format(event[1]))
                    messagebox.showerror("下载失败", event[1], parent=root)
                elif kind == "done":
                    size, chunks = event[1]
                    worker_running["transfer"] = False
                    start_button.configure(state="normal")
                    status_text.set("下载完成：{} 字节，{} 个分片。".format(size, chunks))
                    messagebox.showinfo(
                        "下载完成",
                        "固件已写入设备 Flash 缓冲区。\n大小：{} 字节\n分片：{}".format(
                            size, chunks
                        ),
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
