#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
extract_nvm_config.py
=====================
从 ODrive 整片 1MB Flash 的 Intel HEX dump 中提取 NVM 配置区（0x080C0000 / 0x080E0000,
各 128KB, sector 10/11）, 并按固件 nvm.c 的仲裁逻辑找出"当前生效"的配置块,
重打包成可用于 OTA 的 sector 镜像。

复刻的固件逻辑 (Firmware/MotorControl/nvm.c):
  - 每个扇区头部为 allocation table: 每 64-bit 字段 2 bit 状态
      VALID=00, INVALID=01, ERASED=11
  - N_RESERVED = (SIZE>>3)>>5 = 512 个 64-bit 字段 = 4096 字节留给表
  - 数据区从字段 512 (字节 4096) 开始, 追加写
  - NVM_init: 从扇区尾部往前扫, 最后一条非 ERASED 字段的状态 == VALID
    则该扇区生效; sector11 VALID 优先 (read_sector_ = 1)
  - 生效配置块 = 从尾部往前连续的 VALID 字段区间
  - 块尾部 2 字节为 CRC16(poly=0x3d65, init=0xabcd ^ config_version),
    CRC 覆盖逻辑配置块(含 CRC 自己), 不含 NVM 8 字节对齐填充

用法:
  python extract_nvm_config.py [flash_dump.hex ...] [--full]
      [--config-version VERSION]

  不提供 HEX 路径时，读取 config.txt 中独立的 nvm_extract_paths 列表
  --full : 兼容模式, 输出完整 128KB 扇区镜像 (默认输出裁剪版)
  --config-version : 配置结构版本号 (默认 0x0001, 来自 nvm_config.hpp)

输出 (默认裁剪版, 只含有效内容, 未覆盖区域保持擦除态 0xFF):
  <outdir>/<产品名>Config.bin              OTA 参数镜像 (4096B 表 + 数据块, 约 34KB)
  <outdir>/<产品名>Config.hex              同一镜像, Intel HEX 地址从 0x080C0000 开始

产品名取自固件文件名中 "ODrive-fw" 之前的部分, 例如 RunODrive-fw-v0.5.8.hex
会生成 RunConfig.bin 和 RunConfig.hex。
"""

import sys
import os

FLASH_BASE   = 0x08000000
FLASH_SIZE   = 0x100000          # 1MB
SECT10_BASE  = 0x080C0000
SECT11_BASE  = 0x080E0000
SECTOR_SIZE  = 0x20000           # 128KB

N_DATA       = SECTOR_SIZE >> 3          # 16384 个 64-bit 字段
N_RESERVED   = N_DATA >> 5               # 512 个字段(4096 字节)留给分配表
DATA_START   = N_RESERVED                # 数据区起始字段号 (byte offset = field*8)

ERASED, INVALID, VALID = 3, 1, 0
CRC16_POLY   = 0x3D65
CRC16_INIT   = 0xABCD


# ---------------------------------------------------------------- Intel HEX 解析
def parse_ihex(path, flash_base=FLASH_BASE, flash_size=FLASH_SIZE):
    """把 Intel HEX 展开成以 0x08000000 为基址的字节数组, 空白处填 0xFF"""
    img = bytearray(b"\xff" * flash_size)
    base_upper = 0
    covered = set()
    with open(path, "r") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or not line.startswith(":"):
                continue
            count = int(line[1:3], 16)
            addr  = int(line[3:7], 16)
            rtype = int(line[7:9], 16)
            data  = bytes.fromhex(line[9:9 + count * 2])
            # (校验和不检查了, 读出的 dump 一般没问题)
            if rtype == 0x00:                       # data record
                a = base_upper + addr
                off = a - flash_base
                if off < 0 or off + count > flash_size:
                    print(f"  [警告] 行{lineno}: 地址 0x{a:08X} 超出 1MB 范围, 跳过")
                    continue
                img[off:off + count] = data
                covered.add((off, count))
            elif rtype == 0x01:                     # EOF
                break
            elif rtype == 0x02:                     # extended segment address
                base_upper = int(line[9:13], 16) << 4
            elif rtype == 0x04:                     # extended linear address
                base_upper = int(line[9:13], 16) << 16
            elif rtype == 0x03 or rtype == 0x05:    # start address, 忽略
                pass
    return img, covered


def check_coverage(covered, lo_off, hi_off):
    """粗略检查 [lo_off, hi_off) 是否被 hex 数据覆盖"""
    total = 0
    for off, cnt in covered:
        s, e = max(off, lo_off), min(off + cnt, hi_off)
        if e > s:
            total += e - s
    return total


# ---------------------------------------------------------------- CRC16 (与 fibre/crc.hpp 一致)
_crc_table = None
def _build_table():
    global _crc_table
    tbl = []
    for v in range(256):
        r = v << 8
        for _ in range(8):
            r = ((r << 1) ^ CRC16_POLY) & 0xFFFF if (r & 0x8000) else (r << 1) & 0xFFFF
        tbl.append(r)
    _crc_table = tbl

def crc16(data, init):
    """MSB-first CRC16, 无终态异或 -- 与固件 calc_crc16 完全一致"""
    if _crc_table is None:
        _build_table()
    r = init & 0xFFFF
    for b in data:
        r = ((r << 8) & 0xFFFF) ^ _crc_table[((r >> 8) ^ b) & 0xFF]
    return r


# ---------------------------------------------------------------- NVM 仲裁逻辑 (复刻 nvm.c)
def field_state(sector, i):
    """第 i 个 64-bit 字段的分配表状态 (LSB-first, 每字节 4 个字段)"""
    return (sector[i >> 2] >> (2 * (i & 3))) & 3


def scan_sector(sector):
    """复刻 NVM_init 的扫表: 返回 (index, tail_state, block_start, block_len_fields)
       index      : 追加位置 (第一个 ERASED 字段)
       tail_state : 最后一条非 ERASED 字段的状态
       block_*    : 尾部连续 VALID 字段区间 (仅 tail_state == VALID 时有意义)
    """
    states = [field_state(sector, i) for i in range(N_DATA)]

    # 从尾部往前找最后一条非 ERASED 字段
    i = N_DATA - 1
    while i >= DATA_START and states[i] == ERASED:
        i -= 1
    tail_state = states[i] if i >= DATA_START else ERASED
    index = i + 1

    block_start, block_end = None, None
    if tail_state == VALID:
        j = i
        while j > DATA_START and states[j - 1] == VALID:
            j -= 1
        block_start, block_end = j, i + 1
    return index, tail_state, block_start, block_end


def analyze_sector(sector):
    index, tail_state, bs, be = scan_sector(sector)
    info = {
        "append_index": index,
        "tail_state": tail_state,
        "block_start": bs,
        "block_fields": (be - bs) if bs is not None else 0,
    }
    if bs is not None:
        info["block_bytes"] = sector[bs * 8: be * 8]
    return info


# ---------------------------------------------------------------- CRC 边界校验
def find_config_length(block, config_version):
    """返回通过 CRC 的逻辑块长度和尾部填充长度。

    NVM_start_write 按 8 字节向上取整，分配字段末尾的 0xFF 填充不属于
    config 数据，也不参与 CRC。候选填充长度最多为 7 字节。
    """
    matches = []
    for padding in range(min(7, len(block) - 2) + 1):
        if padding and block[-padding:] != b"\xff" * padding:
            continue
        length = len(block) - padding
        if crc16(block[:length], CRC16_INIT ^ config_version) == 0:
            matches.append((length, padding))
    return matches


# ---------------------------------------------------------------- 生成干净的 sector 镜像
def build_sector_image(block_bytes, full=False):
    """构造一个"已提交"状态的 sector 镜像:
       - 分配表: 数据字段 [512, 512+n) 全部标 VALID, 其余 ERASED(0xFF)
       - 数据区: 4096 字节偏移处放 block_bytes
       NVM_init 扫它: 尾状态 = VALID -> 该扇区成为当前读取扇区

       full=False (默认): 裁剪输出 [0, 4096+n*8)。尾部未覆盖区域在烧写时
       保持擦除态 0xFF, 与镜像中 0xFF 语义完全一致 (flash 只能 1->0),
       因此裁剪不影响固件读取。
       full=True: 输出完整 128KB。"""
    n = (len(block_bytes) + 7) >> 3          # 按字段对齐 (NVM_start_write 同样圆整)
    padded = block_bytes + b"\xff" * (n * 8 - len(block_bytes))
    end = SECTOR_SIZE if full else DATA_START * 8 + n * 8
    img = bytearray(b"\xff" * end)
    for f in range(DATA_START, DATA_START + n):
        img[f >> 2] &= ~(0b11 << (2 * (f & 3))) & 0xFF   # 写 00 = VALID
    img[DATA_START * 8: DATA_START * 8 + n * 8] = padded
    return img, n


def self_check_sector_image(img):
    """用同一套扫表逻辑自检镜像 (补齐到 128KB 后分析): tail=VALID 且块起始正确"""
    padded = bytes(img) + b"\xff" * (SECTOR_SIZE - len(img))
    info = analyze_sector(padded)
    return info["tail_state"] == VALID and info["block_start"] == DATA_START


def ihex_record(address, record_type, data=b""):
    """生成一条带校验和的 Intel HEX 记录"""
    record = bytes((len(data), (address >> 8) & 0xFF, address & 0xFF, record_type)) + data
    checksum = (-sum(record)) & 0xFF
    return ":" + record.hex().upper() + f"{checksum:02X}"


def write_ihex(path, data, base_address):
    """将二进制镜像写成带绝对地址的 Intel HEX 文件"""
    if base_address < 0 or base_address + len(data) > 0x100000000:
        raise ValueError("Intel HEX 地址范围超出 32 位")

    lines = []
    current_upper = None
    for offset in range(0, len(data), 16):
        address = base_address + offset
        upper = address >> 16
        if upper != current_upper:
            lines.append(ihex_record(0, 0x04, upper.to_bytes(2, "big")))
            current_upper = upper
        lines.append(ihex_record(address & 0xFFFF, 0x00, data[offset:offset + 16]))
    lines.append(ihex_record(0, 0x01))
    with open(path, "w", encoding="ascii", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


def config_output_stem(hex_path):
    firmware_stem = os.path.splitext(os.path.basename(hex_path))[0]
    product_name = firmware_stem.split("ODrive-fw", 1)[0].rstrip("-_ ")
    if not product_name:
        product_name = firmware_stem
    return product_name + "Config"


def configured_hex_paths():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.abspath(os.path.join(
        script_dir, "..", "..", "configFiles", "版本说明文件", "config.txt"))
    with open(config_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            if key.strip() == "nvm_extract_paths":
                paths = [item.strip().strip("\"'") for item in value.split(",")]
                return [
                    os.path.abspath(os.path.join(script_dir, path))
                    if not os.path.isabs(path) else path
                    for path in paths if path
                ]
    raise ValueError(f"配置文件未定义 nvm_extract_paths: {config_path}")


def extract_firmware(hex_path, outdir, full, config_version):
    if not os.path.isfile(hex_path):
        print(f"[错误] HEX 文件不存在: {hex_path}")
        return False

    img, covered = parse_ihex(hex_path)
    lo = SECT10_BASE - FLASH_BASE
    hi = SECT11_BASE + SECTOR_SIZE - FLASH_BASE
    coverage = check_coverage(covered, lo, hi)
    if coverage < SECTOR_SIZE * 2:
        print(f"[错误] {hex_path}: NVM 区覆盖不足 ({coverage}/{SECTOR_SIZE * 2} B)")
        return False

    sect10 = img[SECT10_BASE - FLASH_BASE:SECT10_BASE - FLASH_BASE + SECTOR_SIZE]
    sect11 = img[SECT11_BASE - FLASH_BASE:SECT11_BASE - FLASH_BASE + SECTOR_SIZE]
    info10 = analyze_sector(sect10)
    info11 = analyze_sector(sect11)
    active = info11 if info11["tail_state"] == VALID else info10
    if active["block_start"] is None:
        print(f"[错误] {hex_path}: 找不到有效 NVM 配置块")
        return False

    block = bytes(active["block_bytes"])
    matches = find_config_length(block, config_version)
    crc_detail = ""
    if len(matches) == 1:
        logical_length, padding = matches[0]
        crc_detail = f", CRC通过 {logical_length} B (+{padding} B填充)"
    else:
        print(f"[警告] {hex_path}: CRC未能唯一验证，请确认 config_version=0x{config_version:04x}")

    sector_image, n_fields = build_sector_image(block, full=full)
    if not self_check_sector_image(sector_image):
        print(f"[错误] {hex_path}: 镜像自检失败")
        return False

    os.makedirs(outdir, exist_ok=True)
    output_stem = config_output_stem(hex_path)
    bin_path = os.path.join(outdir, output_stem + ".bin")
    output_hex_path = os.path.join(outdir, output_stem + ".hex")
    with open(bin_path, "wb") as f:
        f.write(sector_image)
    write_ihex(output_hex_path, sector_image, SECT10_BASE)

    print(f"[完成] {output_stem}: {len(sector_image)} B, {n_fields} fields{crc_detail}")
    print(f"        {bin_path}")
    print(f"        {output_hex_path}")
    return True


# ---------------------------------------------------------------- 主流程
def main():
    import argparse
    parser = argparse.ArgumentParser(description="从 Intel HEX 中提取 ODrive NVM 配置")
    parser.add_argument("hex_paths", nargs="*", help="完整 Flash dump 的 Intel HEX 文件")
    parser.add_argument("--outdir", help="输出目录 (默认分别使用各 HEX 文件所在目录)")
    parser.add_argument("--full", action="store_true", help="输出完整 128KB 扇区镜像")
    parser.add_argument("--config-version", type=lambda value: int(value, 0), default=0x0001,
                        help="CRC 配置版本号 (默认: 0x0001)")
    args = parser.parse_args()
    if not 0 <= args.config_version <= 0xFFFF:
        parser.error("--config-version 必须在 0 到 0xffff 之间")

    hex_paths = args.hex_paths or configured_hex_paths()
    if not hex_paths:
        parser.error("未提供 HEX 文件，且 nvm_extract_paths 配置为空")

    success = True
    for path in hex_paths:
        hex_path = os.path.abspath(path)
        outdir = args.outdir or os.path.dirname(hex_path)
        if not extract_firmware(hex_path, outdir, args.full, args.config_version):
            success = False
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
