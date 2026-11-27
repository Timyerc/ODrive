#!/usr/bin/env python3
"""Download ODrive parameter and application images over CAN.

Parameters are applied without reboot. A verified application image triggers
a reboot into the bootloader. Windows defaults to PCAN; other platforms use
SocketCAN.
"""

import argparse
import os
import queue
import struct
import sys
import threading
import time
import zlib


CAN_REQUEST_ID = 0xABC
CAN_RESPONSE_ID = 0xDEF
CHUNK_SIZE = 1024
MAX_FIRMWARE_SIZE = 320 * 1024
FRAME_DATA = 0x01
FRAME_DATA_ACK = 0x02
FRAME_CHUNK_CRC_REQUEST = 0x03
FRAME_CHUNK_CRC_RESPONSE = 0x04
FRAME_IMAGE_CRC_REQUEST = 0x05
FRAME_IMAGE_CRC_RESPONSE = 0x06
FRAME_VERSION_REQUEST = 0x07
FRAME_VERSION_RESPONSE = 0x08
FRAME_APPLICATION_REQUEST = 0x10
FRAME_PARAMETERS_REQUEST = 0x11
FRAME_READY = 0x13
READY_PENDING = 0
READY_SUCCESS = 1
READY_REJECTED = 2
TRANSFER_SUCCESS = 1
IMAGE_TYPES = {"app": 1, "parameters": 2}
REQUEST_OPCODES = {"app": FRAME_APPLICATION_REQUEST,
                   "parameters": FRAME_PARAMETERS_REQUEST}

READY_STATUS_NAMES = {
    READY_PENDING: "preparing",
    READY_SUCCESS: "ready",
    READY_REJECTED: "request rejected",
}

def make_message(can, payload):
    return can.Message(
        arbitration_id=CAN_REQUEST_ID,
        is_extended_id=True,
        data=payload,
    )


def send_control(bus, can, payload, log):
    bus.send(make_message(can, payload))
    if log:
        log("TX 0xABC EXT [{}] {}".format(len(payload), payload.hex(" ")))


def wait_for_version_response(bus, timeout, log=None):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("timed out waiting for firmware version response")
        message = bus.recv(remaining)
        if message is None or message.is_error_frame or message.is_remote_frame:
            continue
        if (message.arbitration_id != CAN_RESPONSE_ID or not message.is_extended_id or
                not message.data or message.data[0] != FRAME_VERSION_RESPONSE):
            continue
        if len(message.data) != 5:
            raise RuntimeError(
                "invalid firmware version response: {}".format(message.data.hex(" "))
            )
        version = tuple(message.data[1:5])
        if log:
            log("RX VERSION 0xDEF EXT [5] {} | {}.{}.{}{}".format(
                message.data.hex(" "), version[0], version[1], version[2],
                "-dev" if version[3] else ""))
        return version


def query_firmware_version(interface, channel, bitrate, timeout=5.0,
                           retries=5, can_module=None, log_callback=None):
    if can_module is None:
        try:
            import can as can_module
        except ImportError as error:
            raise RuntimeError(
                "缺少 python-can，请运行：python -m pip install python-can"
            ) from error

    with can_module.Bus(interface=interface, channel=channel, bitrate=bitrate) as bus:
        for attempt in range(retries + 1):
            send_control(bus, can_module, bytes([FRAME_VERSION_REQUEST]), log_callback)
            try:
                return wait_for_version_response(bus, timeout, log_callback)
            except TimeoutError:
                if attempt == retries:
                    raise
    raise RuntimeError("firmware version query retries exhausted")


def wait_for_ready(bus, timeout, log=None):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            if log:
                log("ready response timeout")
            raise TimeoutError("timed out waiting for device ready response")
        message = bus.recv(remaining)
        if message is None or message.is_error_frame or message.is_remote_frame:
            continue
        if (message.arbitration_id != CAN_RESPONSE_ID or not message.is_extended_id or
                len(message.data) != 8 or message.data[0] != FRAME_READY):
            continue
        status = message.data[1]
        chunk_count = struct.unpack_from("<H", message.data, 2)[0]
        if log:
            log("RX READY 0xDEF EXT [8] {} | {} chunks={}".format(
                message.data.hex(" "),
                READY_STATUS_NAMES.get(status, "status {}".format(status)),
                chunk_count))
        if status != READY_PENDING:
            return status, chunk_count


def request_ready(bus, can, request, timeout, log=None):
    deadline = time.monotonic() + 120.0
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("等待驱动器停机并准备 OTA 超时")
        send_control(bus, can, request, log)
        try:
            response = wait_for_ready(bus, min(timeout, remaining), log)
        except TimeoutError:
            continue
        status, chunk_count = response
        if status == READY_SUCCESS:
            return chunk_count
        if status == READY_REJECTED:
            raise RuntimeError("驱动器拒绝 OTA 请求")
        raise RuntimeError("驱动器返回未知就绪状态：{}".format(status))


def wait_for_frame_ack(bus, frame_index, timeout, log=None):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                "等待 CAN 帧 {} 的驱动器应答超时".format(frame_index)
            )
        message = bus.recv(remaining)
        if message is None or message.is_error_frame or message.is_remote_frame:
            continue
        if (message.arbitration_id != CAN_RESPONSE_ID or not message.is_extended_id or
                len(message.data) != 7 or message.data[0] != FRAME_DATA_ACK or
                message.data[1] != frame_index):
            continue
        if log:
            log("RX ACK 0xDEF EXT [7] {} | frame={} received".format(
                message.data.hex(" "), frame_index))
        return


def send_data_frame(bus, can, frame_index, payload, args, log_callback=None):
    message = make_message(can, payload)
    for attempt in range(args.retries + 1):
        bus.send(message)
        if log_callback and attempt:
            log_callback("重发帧内数据 frame={} attempt={}".format(
                frame_index, attempt + 1))
        try:
            wait_for_frame_ack(bus, frame_index, args.timeout, log_callback)
        except TimeoutError:
            if attempt == args.retries:
                raise
            continue
        return
    raise RuntimeError("数据帧 {} 重试次数耗尽".format(frame_index))


def wait_for_crc_response(bus, response_opcode, timeout, log=None):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("等待 CRC32 应答超时")
        message = bus.recv(remaining)
        if message is None or message.is_error_frame or message.is_remote_frame:
            continue
        if (message.arbitration_id != CAN_RESPONSE_ID or not message.is_extended_id or
                not message.data or message.data[0] != response_opcode):
            continue
        if len(message.data) != 5:
            raise RuntimeError(
                "驱动器 CRC32 查询失败：{}".format(message.data.hex(" "))
            )
        checksum = struct.unpack_from("<I", message.data, 1)[0]
        if log:
            log("RX CRC 0xDEF EXT [5] {} | CRC32=0x{:08X}".format(
                message.data.hex(" "), checksum))
        return checksum


def send_crc_request(bus, can, opcode, response_opcode, checksum, args,
                     log_callback=None):
    payload = bytes([opcode]) + struct.pack("<I", checksum)
    for attempt in range(args.retries + 1):
        send_control(bus, can, payload, log_callback)
        try:
            return wait_for_crc_response(
                bus, response_opcode, args.timeout, log_callback)
        except TimeoutError:
            if attempt == args.retries:
                raise
    raise RuntimeError("CRC32 查询重试次数耗尽")


def send_chunk(bus, can, sequence, image_size, chunk, args, log_callback=None,
               interference_queue=None):
    checksum = zlib.crc32(chunk) & 0xFFFFFFFF
    valid_chunk_size = min(CHUNK_SIZE, max(0, image_size - sequence * CHUNK_SIZE))

    def consume_interference():
        if interference_queue is None:
            return False
        try:
            interference_queue.get_nowait()
            return True
        except queue.Empty:
            return False

    for attempt in range(args.retries + 1):
        if log_callback:
            log_callback("chunk={} CRC32=0x{:08X} attempt={}/{}".format(
                sequence, checksum, attempt + 1, args.retries + 1))

        for frame_index, offset in enumerate(range(0, CHUNK_SIZE, 6)):
            payload = chunk[offset:offset + 6].ljust(6, b"\xFF")
            frame_data = bytearray(payload)
            valid_bytes = min(6, max(0, valid_chunk_size - offset))
            if valid_bytes and consume_interference():
                frame_data[0] ^= 0x01
                if log_callback:
                    log_callback(
                        "SIMULATED INTERFERENCE: flipped data bit in chunk={} frame={}; "
                        "original chunk CRC retained to trigger retry".format(
                            sequence, frame_index
                        )
                    )
            send_data_frame(
                bus, can, frame_index,
                bytes([FRAME_DATA, frame_index]) + frame_data,
                args, log_callback)
            if args.frame_delay:
                time.sleep(args.frame_delay)

        actual_crc = send_crc_request(
            bus, can, FRAME_CHUNK_CRC_REQUEST, FRAME_CHUNK_CRC_RESPONSE, checksum,
            args, log_callback)
        if actual_crc == checksum:
            if attempt and log_callback:
                log_callback(
                    "Chunk {} accepted after {} retry attempt(s)".format(
                        sequence, attempt
                    )
                )
            return TRANSFER_SUCCESS
        if attempt < args.retries:
            message = "Chunk {} rejected ({}); retrying".format(
                sequence,
                "CRC32 mismatch: device 0x{:08X}, host 0x{:08X}".format(
                    actual_crc, checksum))
            if log_callback:
                log_callback(message)
            else:
                print(message, file=sys.stderr)
            continue
        raise RuntimeError(
            "分片 {} CRC32 不匹配：本地 0x{:08X}，设备 0x{:08X}".format(
                sequence, checksum, actual_crc)
        )

    raise RuntimeError("chunk {} failed after retries".format(sequence))


def read_image(firmware_path, image_type):
    basename = os.path.basename(os.fspath(firmware_path))
    filename = basename.lower()
    required_name = {"app": "firmware", "parameters": "config"}.get(image_type)
    if required_name is None:
        raise ValueError("请选择应用程序或参数")
    if os.path.splitext(filename)[1] != ".bin":
        raise ValueError("镜像文件必须是 .bin 格式")
    if required_name not in filename:
        label = "Firmware" if image_type == "app" else "Config"
        raise ValueError("文件名必须包含 {}：{}".format(label, basename))
    with open(firmware_path, "rb") as firmware_file:
        firmware = firmware_file.read(MAX_FIRMWARE_SIZE + 1)
    if not firmware:
        raise ValueError("固件文件为空")
    if image_type == "app":
        if len(firmware) > MAX_FIRMWARE_SIZE:
            raise ValueError("应用镜像超出 320 KiB Flash 分区")
        if len(firmware) < 8:
            raise ValueError("应用镜像缺少复位向量入口")
        entry = struct.unpack_from("<I", firmware, 4)[0]
        if not (entry & 1 and 0x08010000 <= (entry & ~1) < 0x08060000):
            raise ValueError("应用复位入口必须位于 APPLICATION_START 所在应用分区")
    elif len(firmware) > 128 * 1024:
        raise ValueError("参数镜像超出 128 KiB Flash 分区")
    return firmware


def image_parameters(firmware):
    size = len(firmware)
    return size, zlib.crc32(firmware) & 0xFFFFFFFF, (size + CHUNK_SIZE - 1) // CHUNK_SIZE


def transfer_firmware(firmware_path, interface, channel, bitrate,
                      image_type="app", timeout=5.0,
                      retries=5, frame_delay=0.001, can_module=None,
                      progress_callback=None, log_callback=None, *,
                      image_size=None, image_crc=None, chunk_count=None,
                      interference_queue=None):
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
        request = bytes([REQUEST_OPCODES[image_type]]) + image_size.to_bytes(3, "little")
        ready_chunk_count = request_ready(
            bus, can_module, request, timeout, log_callback
        )
        if ready_chunk_count != chunk_count:
            raise RuntimeError(
                "驱动器就绪片数不匹配：本地 {}，设备 {}".format(
                    chunk_count, ready_chunk_count
                )
            )
        for sequence in range(chunk_count):
            offset = sequence * CHUNK_SIZE
            chunk = firmware[offset:offset + CHUNK_SIZE].ljust(CHUNK_SIZE, b"\xFF")
            status = send_chunk(
                bus, can_module, sequence, len(firmware), chunk, transfer_args,
                log_callback=log_callback,
                interference_queue=interference_queue,
            )
            if progress_callback:
                progress_callback(sequence + 1, chunk_count, status)
        device_crc = send_crc_request(
            bus, can_module, FRAME_IMAGE_CRC_REQUEST,
            FRAME_IMAGE_CRC_RESPONSE, image_crc, transfer_args, log_callback)
    if device_crc != image_crc:
        raise RuntimeError(
            "设备整包 CRC32 不匹配：发送 0x{:08X}，设备读回 0x{:08X}".format(
                image_crc, device_crc
            )
        )
    return len(firmware), chunk_count, device_crc


def transfer_images_in_order(parameters_path, application_path, interface, channel,
                             bitrate, can_module=None, progress_callback=None,
                             log_callback=None, image_parameters_by_type=None,
                             timeout=5.0, retries=5, frame_delay=0.001,
                             interference_queue=None):
    results = {}
    for image_type, path in (("parameters", parameters_path), ("app", application_path)):
        options = (image_parameters_by_type or {}).get(image_type, {})

        def report_progress(current, total, status, selected_type=image_type):
            if progress_callback:
                progress_callback(selected_type, current, total, status)

        results[image_type] = transfer_firmware(
            path, interface, channel, bitrate,
            image_type=image_type,
            can_module=can_module,
            progress_callback=report_progress,
            log_callback=log_callback,
            timeout=timeout,
            retries=retries,
            frame_delay=frame_delay,
            interference_queue=interference_queue,
            **options
        )
        if log_callback:
            log_callback("{} 镜像 CRC32 校验通过：0x{:08X}".format(
                image_type, results[image_type][2]))
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Send a firmware image to the ODrive Flash buffer over CAN"
    )
    parser.add_argument("firmware", nargs="?", help="firmware binary file")
    default_interface = "pcan" if sys.platform == "win32" else "socketcan"
    parser.add_argument("--interface", default=default_interface,
                        help="python-can interface (default: {})".format(default_interface))
    parser.add_argument("--channel", required=True,
                        help="CAN channel, for example can0 or PCAN_USBBUS1")
    parser.add_argument("--type", choices=IMAGE_TYPES, default="app",
                        help="target image: app (default) or parameters")
    parser.add_argument("--auto", action="store_true",
                        help="download parameters first, then application")
    parser.add_argument("--read-version", action="store_true",
                        help="read the connected drive's firmware version")
    parser.add_argument("--parameters-image", help="parameter Config .bin for --auto")
    parser.add_argument("--application-image", help="application Firmware .bin for --auto")
    for option in ("size", "crc32", "chunks"):
        parser.add_argument("--" + option, type=lambda value: int(value, 0),
                            help="expected image {}; calculated if omitted".format(option))
    parser.add_argument("--bitrate", type=int, default=100000,
                        help="CAN bitrate in bit/s (default: 100000)")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="acknowledgement timeout in seconds (default: 5)")
    parser.add_argument("--retries", type=int, default=5,
                        help="maximum retries per CAN frame or 1 KiB chunk (default: 5)")
    parser.add_argument("--frame-delay", type=float, default=0.001,
                        help="delay between data frames in seconds (default: 0.001)")
    args = parser.parse_args(argv)

    if args.timeout <= 0 or args.retries < 0 or args.frame_delay < 0:
        parser.error("timeout must be positive and retries/frame-delay non-negative")

    try:
        if args.read_version:
            if (args.firmware or args.auto or args.parameters_image or
                    args.application_image or args.size is not None or
                    args.crc32 is not None or args.chunks is not None):
                parser.error("--read-version cannot be combined with firmware download options")
            version = query_firmware_version(
                args.interface, args.channel, args.bitrate,
                timeout=args.timeout, retries=args.retries, log_callback=print)
            print("Firmware version {}.{}.{}{}".format(
                version[0], version[1], version[2],
                "-dev" if version[3] else ""))
            return 0
        elif args.auto:
            if args.firmware or not args.parameters_image or not args.application_image:
                parser.error("--auto requires --parameters-image and --application-image, without firmware positional")
            results = transfer_images_in_order(
                args.parameters_image, args.application_image,
                args.interface, args.channel, args.bitrate,
                log_callback=print,
                timeout=args.timeout,
                retries=args.retries,
                frame_delay=args.frame_delay,
                progress_callback=lambda kind, current, total, status: print(
                    "{} chunk {}/{} {}".format(
                            kind, current, total,
                            "complete" if status == TRANSFER_SUCCESS else "saved"
                    ), flush=True),
            )
        else:
            if not args.firmware or args.parameters_image or args.application_image:
                parser.error("provide a firmware file, or use --auto with both image paths")
            firmware_size, chunk_count, device_crc = transfer_firmware(
                args.firmware, args.interface, args.channel, args.bitrate,
                image_type=args.type,
                image_size=args.size, image_crc=args.crc32, chunk_count=args.chunks,
                log_callback=print,
                timeout=args.timeout,
                retries=args.retries,
                frame_delay=args.frame_delay,
                progress_callback=lambda current, total, status: print(
                    "Chunk {}/{} {}".format(
                        current, total,
                        "complete" if status == TRANSFER_SUCCESS else "saved"
                    ),
                    flush=True,
                ),
            )
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(str(error))

    if args.auto:
        print("Parameter and application CRC32 verified. The board is rebooting "
              "to install the application image.")
    else:
        print("CRC32 verified: 0x{:08X}; {} bytes, {} chunks.".format(
            device_crc, firmware_size, chunk_count))
        if args.type == "app":
            print("The board is rebooting to install the application image.")
        else:
            print("Parameters were installed without rebooting the board.")
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
    root.geometry("780x760")
    root.minsize(680, 620)
    events = queue.Queue()
    interference_queue = queue.Queue(maxsize=1)
    interference_requested = {"value": False}

    image_paths = {
        "app": tk.StringVar(),
        "parameters": tk.StringVar(),
    }
    image_summaries = {}
    image_progress = {}
    download_buttons = {}
    image_fields = {}
    auto_button = None
    interference_button = None
    interface_name = tk.StringVar(value=default_interface)
    channel_name = tk.StringVar(
        value="PCAN_USBBUS1" if default_interface == "pcan" else ""
    )
    bitrate_value = tk.StringVar(value="100000")
    status_text = tk.StringVar(value="请选择固件并确认 CAN 参数。")

    main_frame = ttk.Frame(root, padding=14)
    main_frame.pack(fill="both", expand=True)
    main_frame.columnconfigure(1, weight=1)

    def choose_image(image_type):
        display_name = "应用固件（文件名需包含 Firmware）" if image_type == "app" \
            else "参数镜像（文件名需包含 Config）"
        path = filedialog.askopenfilename(
            title="选择" + display_name,
            filetypes=(("固件二进制文件", "*.bin"), ("所有文件", "*.*")),
        )
        if path:
            image_paths[image_type].set(path)
            try:
                image = read_image(path, image_type)
                size, checksum, chunks = image_parameters(image)
            except (OSError, ValueError) as error:
                image_summaries[image_type].set("校验失败：{}".format(error))
                append_log("{} 镜像校验失败：{}".format(display_name, error))
            else:
                image_summaries[image_type].set(
                    "{} 字节 | CRC32=0x{:08X} | {} 片".format(size, checksum, chunks)
                )
                append_log("{} 镜像初步校验通过：{}".format(display_name, path))

    image_frame = ttk.LabelFrame(main_frame, text="待下载镜像", padding=8)
    image_frame.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 8))
    image_frame.columnconfigure(1, weight=1)
    for row, (image_type, title) in enumerate((
        ("app", "应用镜像（名称包含 Firmware）"),
        ("parameters", "参数镜像（名称包含 Config）"),
    )):
        ttk.Label(image_frame, text=title).grid(row=row * 4, column=0, sticky="w")
        ttk.Entry(image_frame, textvariable=image_paths[image_type]).grid(
            row=row * 4, column=1, sticky="ew", padx=8, pady=3
        )
        ttk.Button(image_frame, text="选择 .bin",
                   command=lambda selected=image_type: choose_image(selected)).grid(
                       row=row * 4, column=2, padx=(4, 0)
                   )
        summary = tk.StringVar(value="尚未选择镜像")
        image_summaries[image_type] = summary
        ttk.Label(image_frame, textvariable=summary).grid(
            row=row * 4 + 1, column=0, columnspan=3, sticky="w"
        )
        progress = ttk.Progressbar(image_frame, mode="determinate")
        progress.grid(row=row * 4 + 2, column=0, columnspan=2, sticky="ew", pady=3)
        image_progress[image_type] = progress
        button = ttk.Button(
            image_frame,
            text="开始下载应用" if image_type == "app" else "开始下载参数",
            command=lambda selected=image_type: start_transfer(selected),
        )
        button.grid(row=row * 4 + 2, column=2, padx=(8, 0))
        download_buttons[image_type] = button
        fields_frame = ttk.Frame(image_frame)
        fields_frame.grid(row=row * 4 + 3, column=0, columnspan=3, sticky="ew", pady=(1, 5))
        image_fields[image_type] = {}
        for column, (key, label) in enumerate((
            ("image_size", "大小（字节）"),
            ("image_crc", "整包 CRC-32"),
            ("chunk_count", "分片数"),
        )):
            variable = tk.StringVar()
            image_fields[image_type][key] = variable
            ttk.Label(fields_frame, text=label).grid(row=0, column=column * 2, sticky="w")
            ttk.Entry(fields_frame, textvariable=variable, width=13).grid(
                row=0, column=column * 2 + 1, padx=(3, 10)
            )
        ttk.Button(fields_frame, text="一键计算",
                   command=lambda selected=image_type: calculate_image(selected)).grid(
                       row=0, column=6, padx=(4, 0)
                   )

    auto_button = ttk.Button(
        image_frame, text="自动下载：参数 → 应用",
        command=lambda: start_auto_transfer(),
    )
    auto_button.grid(row=8, column=0, columnspan=3, pady=(4, 8))
    interference_button = ttk.Button(
        image_frame, text="模拟数据干扰（可重复点击）",
        state="disabled", command=lambda: request_interference(),
    )
    interference_button.grid(row=9, column=0, columnspan=3, pady=(0, 6))

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

    status_label = ttk.Label(main_frame, textvariable=status_text, wraplength=680)
    status_label.grid(row=4, column=0, columnspan=3, sticky="w", pady=4)

    instructions = (
        "使用说明\n"
        "1. 分别选择应用和参数 .bin：应用不超过 320 KiB，参数不超过 128 KiB；文件名需分别包含 Firmware 和 Config。\n"
        "2. 选择 CAN 接口并扫描端口，也可以手动输入通道名；选择与设备一致的波特率。\n"
        "3. 请求后由驱动器自行停机并通过 0x13 回复就绪状态。每个 0x01 数据帧都需收到对应 0x02 应答；每片通过 0x03 查询、0x04 回复分片 CRC，整包通过 0x05 查询、0x06 回复 CRC；CAN 请求 ID 为 0xABC，响应 ID 为 0xDEF。\n"
        "4. 可分别下载，或使用自动下载按“参数→应用”顺序执行。参数写入后不重启；应用 CRC 校验成功后复位进入 bootloader。\n"
        "5. 下载期间可重复按“模拟数据干扰”，每次破坏一个数据帧以验证分片 CRC 不匹配和整片重传；上一次重传成功后按钮会重新启用。\n\n"
        "Python 支持包与驱动\n"
        "• 必装：python -m pip install python-can\n"
        "• PCAN-USB：安装 PEAK PCAN 驱动/PCAN-Basic；接口选 pcan，端口通常为 PCAN_USBBUS1。\n"
        "• SLCAN 串口设备：另需 python -m pip install pyserial；端口通常为 COMx。\n"
        "• SocketCAN：仅 Linux；先配置好系统 CAN 接口。\n"
        "• Tkinter 通常随 Windows Python 提供；Linux 若缺少，请安装 python3-tk。"
    )
    tabs = ttk.Notebook(main_frame)
    tabs.grid(row=5, column=0, columnspan=3, sticky="nsew")
    main_frame.rowconfigure(5, weight=1)
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

    def request_interference():
        if worker_running["transfer"] is None:
            return
        try:
            interference_queue.put_nowait(True)
        except queue.Full:
            status_text.set("已有一次模拟干扰等待注入，请等当前分片重传后再试。")
        else:
            interference_requested["value"] = True
            interference_button.configure(state="disabled")
            status_text.set("模拟干扰已排队，将破坏当前分片的有效数据并触发 CRC 重传。")
            append_log(status_text.get())

    def calculate_image(image_type):
        path = image_paths[image_type].get().strip()
        if not path:
            messagebox.showerror("缺少镜像", "请先选择对应的 .bin 镜像。", parent=root)
            return
        try:
            size, checksum, chunks = image_parameters(read_image(path, image_type))
        except (OSError, ValueError) as error:
            image_summaries[image_type].set("校验失败：{}".format(error))
            messagebox.showerror("无法计算镜像参数", str(error), parent=root)
            return
        image_fields[image_type]["image_size"].set(str(size))
        image_fields[image_type]["image_crc"].set("0x{:08X}".format(checksum))
        image_fields[image_type]["chunk_count"].set(str(chunks))
        status_text.set("{}镜像参数已计算。".format(
            "应用" if image_type == "app" else "参数"))

    worker_running = {"transfer": None, "detection": False}

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

    def prepare_image_transfer(image_type):
        path = image_paths[image_type].get().strip()
        if not path:
            label = "应用 Firmware" if image_type == "app" else "参数 Config"
            raise ValueError("请先选择{}名称的 .bin 文件。".format(label))
        firmware = read_image(path, image_type)
        size, checksum, chunks = image_parameters(firmware)
        image_summaries[image_type].set(
            "{} 字节 | CRC32=0x{:08X} | {} 片".format(size, checksum, chunks)
        )
        options = {}
        for key, variable in image_fields[image_type].items():
            text = variable.get().strip()
            if not text:
                options[key] = None
                continue
            try:
                value = int(text, 0)
                if not 0 <= value <= 0xFFFFFFFF:
                    raise ValueError()
            except ValueError:
                raise ValueError("{} 参数必须为 32 位无符号整数。".format(key))
            options[key] = value
        return {"path": path, "options": options, "chunks": chunks}

    def start_transfer(image_type):
        start_transfer_batch((image_type,))

    def start_auto_transfer():
        start_transfer_batch(("parameters", "app"))

    def start_transfer_batch(image_types):
        if worker_running["transfer"] is not None:
            return
        try:
            images = {kind: prepare_image_transfer(kind) for kind in image_types}
            channel = channel_name.get().strip()
            if not channel:
                raise ValueError("请选择 CAN 端口，或手动输入通道名。")
            bitrate = int(bitrate_value.get())
            config = {
                "interface": interface_name.get(),
                "channel": channel,
                "bitrate": bitrate,
            }
        except (OSError, ValueError) as error:
            messagebox.showerror("下载参数无效", str(error), parent=root)
            return

        auto = len(image_types) == 2
        transfer_kind = "auto" if auto else image_types[0]
        worker_running["transfer"] = transfer_kind
        interference_requested["value"] = False
        for button in download_buttons.values():
            button.configure(state="disabled")
        auto_button.configure(state="disabled")
        interference_button.configure(state="normal")
        for image_type, image in images.items():
            image_progress[image_type].configure(maximum=image["chunks"], value=0)
        status_text.set("正在按参数→应用顺序下载..." if auto else
                        "正在下载{}镜像...".format(
                            "应用" if transfer_kind == "app" else "参数"))

        def transfer():
            try:
                import can as can_module
                progress = lambda kind, current, total, _status: events.put(
                    ("progress", kind, current, total)
                )
                logger = lambda message: events.put(("log", message))
                if auto:
                    result = transfer_images_in_order(
                        images["parameters"]["path"],
                        images["app"]["path"],
                        config["interface"],
                        config["channel"],
                        config["bitrate"],
                        can_module=can_module,
                        progress_callback=progress,
                        log_callback=logger,
                        image_parameters_by_type={
                            kind: image["options"] for kind, image in images.items()
                        },
                        interference_queue=interference_queue,
                    )
                    events.put(("auto_done", result))
                else:
                    image_type = transfer_kind
                    result = transfer_firmware(
                        images[image_type]["path"],
                        config["interface"],
                        config["channel"],
                        config["bitrate"],
                        image_type=image_type,
                        **images[image_type]["options"],
                        can_module=can_module,
                        interference_queue=interference_queue,
                        progress_callback=lambda current, total, _status: events.put(
                            ("progress", image_type, current, total)
                        ),
                        log_callback=logger,
                    )
                    events.put(("done", image_type, result))
            except ImportError as error:
                events.put(("transfer_error", transfer_kind,
                            "缺少 python-can：python -m pip install python-can\n{}".format(error)))
            except (can_module.CanError, OSError, ValueError, RuntimeError) as error:
                events.put(("transfer_error", transfer_kind, str(error)))

        threading.Thread(target=transfer, daemon=True).start()

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
                    image_type, current, total = event[1], event[2], event[3]
                    image_progress[image_type].configure(maximum=total, value=current)
                    status_text.set("{}镜像已保存分片 {}/{}。".format(
                        "应用" if image_type == "app" else "参数", current, total))
                elif kind == "log":
                    append_log(event[1])
                    if (interference_requested["value"] and
                            interference_queue.empty() and
                            "accepted after" in event[1] and
                            worker_running["transfer"] is not None):
                        interference_requested["value"] = False
                        interference_button.configure(state="normal")
                        status_text.set(
                            "模拟干扰分片已成功重传，可再次点击进行验证。"
                        )
                elif kind == "transfer_error":
                    transfer_kind, error = event[1], event[2]
                    worker_running["transfer"] = None
                    interference_requested["value"] = False
                    for button in download_buttons.values():
                        button.configure(state="normal")
                    auto_button.configure(state="normal")
                    interference_button.configure(state="disabled")
                    try:
                        interference_queue.get_nowait()
                    except queue.Empty:
                        pass
                    else:
                        append_log("传输已结束，未消费的模拟干扰请求已取消。")
                    label = "自动下载" if transfer_kind == "auto" else \
                        ("应用镜像" if transfer_kind == "app" else "参数镜像")
                    status_text.set("{}失败：{}".format(label, error))
                    append_log(status_text.get())
                    messagebox.showerror("下载失败", error, parent=root)
                elif kind == "done":
                    image_type, result = event[1], event[2]
                    size, chunks, device_crc = result
                    worker_running["transfer"] = None
                    interference_requested["value"] = False
                    for button in download_buttons.values():
                        button.configure(state="normal")
                    auto_button.configure(state="normal")
                    interference_button.configure(state="disabled")
                    try:
                        interference_queue.get_nowait()
                    except queue.Empty:
                        pass
                    else:
                        append_log("传输已结束，未消费的模拟干扰请求已取消。")
                    image_name = "应用" if image_type == "app" else "参数"
                    if image_type == "app":
                        status_text.set(
                            "{}镜像 CRC32 已确认，设备正在复位进入 bootloader。".format(image_name)
                        )
                    else:
                        status_text.set(
                            "{}镜像 CRC32 已确认，参数已写入 A 区，设备未重启。".format(image_name)
                        )
                    append_log("{} | 大小：{} 字节 | 分片：{} | CRC32=0x{:08X}".format(
                        status_text.get(), size, chunks, device_crc))
                    messagebox.showinfo(
                        "下载完成",
                        "{}\n大小：{} 字节\n分片：{}\nCRC32：0x{:08X}".format(
                            status_text.get(), size, chunks, device_crc),
                        parent=root,
                    )
                elif kind == "auto_done":
                    results = event[1]
                    parameter_size, parameter_chunks, parameter_crc = results["parameters"]
                    app_size, app_chunks, app_crc = results["app"]
                    worker_running["transfer"] = None
                    interference_requested["value"] = False
                    for button in download_buttons.values():
                        button.configure(state="normal")
                    auto_button.configure(state="normal")
                    interference_button.configure(state="disabled")
                    try:
                        interference_queue.get_nowait()
                    except queue.Empty:
                        pass
                    else:
                        append_log("传输已结束，未消费的模拟干扰请求已取消。")
                    status_text.set(
                        "参数与应用镜像 CRC32 均已确认；设备正在复位进入 bootloader 更新应用。"
                    )
                    append_log("{} 参数={}B/{}片 CRC32=0x{:08X}; 应用={}B/{}片 CRC32=0x{:08X}".format(
                        status_text.get(), parameter_size, parameter_chunks, parameter_crc,
                        app_size, app_chunks, app_crc))
                    messagebox.showinfo(
                        "自动下载完成",
                        "参数镜像已写入，应用镜像校验通过。\n"
                        "设备正在复位进入 bootloader。\n"
                        "参数 CRC32：0x{:08X}\n应用 CRC32：0x{:08X}".format(
                            parameter_crc, app_crc),
                        parent=root,
                    )
        except queue.Empty:
            pass
        root.after(100, process_events)

    def close_window():
        if worker_running["transfer"] is not None:
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
