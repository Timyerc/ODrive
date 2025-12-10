#!/usr/bin/env python3
"""
ODrive固件拼接脚本
功能：
1. 读取ODriveFirmware.hex和MotorPara.hex文件
2. 从tagName文件中提取版本号
3. 生成新的HEX文件,t命名为ODrive-fw-v{version}.hex
4. 移除0x80FFFFF地址之后的所有数据
"""

import re
import os
import sys
import subprocess
import platform

DEBUG = False

# 地址阈值 - 移除这个地址之后的所有数据
ADDRESS_THRESHOLD = 0x80FFFFF  # 0x08000000 - 1

def parse_hex_line(line):
    """
    解析HEX文件的一行记录
    返回: (地址, 记录类型, 数据长度, 数据, 校验和)
    """
    line = line.strip()
    if not line.startswith(':'):
        return None
    
    try:
        byte_count = int(line[1:3], 16)
        address = int(line[3:7], 16)
        record_type = int(line[7:9], 16)
        data = line[9:9+byte_count*2]
        checksum = int(line[9+byte_count*2:], 16)
        
        return {
            'address': address,
            'record_type': record_type,
            'byte_count': byte_count,
            'data': data,
            'checksum': checksum,
            'full_line': line
        }
    except:
        return None

def calculate_checksum(data):
    """
    计算HEX记录的校验和
    """
    total = 0
    for i in range(0, len(data), 2):
        total += int(data[i:i+2], 16)
    
    # 取补码
    checksum = (-total) & 0xFF
    return checksum

def should_keep_record(record, extended_address=0x0000):
    """
    判断是否应该保留该记录（地址是否在ADDRESS_THRESHOLD之前）
    """
    if record is None:
        return False
    
    # 如果是扩展线性地址记录，更新当前的高位地址
    if record['record_type'] == 4:
        # 扩展线性地址记录，地址范围是0x0000-0xFFFF
        return True
    
    # 如果是结束记录，保留
    if record['record_type'] == 1:
        return True
    
    # 计算完整地址 = 扩展地址 * 0x10000 + 偏移地址
    full_address = (extended_address << 16) + record['address']
    
    # 只保留地址小于等于ADDRESS_THRESHOLD的记录
    return full_address <= ADDRESS_THRESHOLD

def filter_hex_file(input_file):
    """
    过滤HEX文件，移除ADDRESS_THRESHOLD地址之后的所有数据
    返回过滤后的行列表
    """
    filtered_lines = []
    current_extended_address = 0x0000
    removed_count = 0
    kept_count = 0
    
    try:
        with open(input_file, 'r') as f:
            lines = f.readlines()
        
        for line in lines:
            line = line.strip()
            record = parse_hex_line(line)
            
            if record:
                # 更新扩展地址
                if record['record_type'] == 4:
                    current_extended_address = int(record['data'][0:4], 16)
                
                # 判断是否保留该记录
                if should_keep_record(record, current_extended_address):
                    filtered_lines.append(line)
                    kept_count += 1
                    
                    # 如果是数据记录，输出地址信息用于调试
                    if record['record_type'] == 0:
                        full_address = (current_extended_address << 16) + record['address']
                        # if full_address > ADDRESS_THRESHOLD - 0x1000:  # 只显示接近阈值的数据
                            # print(f"  保留地址: 0x{full_address:08X}")
                else:
                    removed_count += 1
                    # 输出被移除的记录信息
                    if record['record_type'] == 0:
                        full_address = (current_extended_address << 16) + record['address']
                        # print(f"  移除地址: 0x{full_address:08X}")
            else:
                # 不是有效的HEX记录，保留原样
                filtered_lines.append(line)
        if DEBUG:
            print(f"  过滤结果: 保留 {kept_count} 条记录，移除 {removed_count} 条记录")
        return filtered_lines
        
    except Exception as e:
        print(f"过滤HEX文件时出错: {e}")
        return None

def merge_hex_files(firmware_file, motorpara_file, output_file):
    """
    合并两个HEX文件，并移除ADDRESS_THRESHOLD地址之后的所有数据
    """
    try:
        if DEBUG:
            # 先过滤固件文件，移除ADDRESS_THRESHOLD之后的数据
            print(f"过滤固件文件，移除0x{ADDRESS_THRESHOLD:08X}地址之后的数据...")
        firmware_lines = filter_hex_file(firmware_file)
        if firmware_lines is None:
            return False
        
        if DEBUG:
            # 过滤电机参数文件
            print(f"过滤电机参数文件，移除0x{ADDRESS_THRESHOLD:08X}地址之后的数据...")
        motorpara_lines = filter_hex_file(motorpara_file)
        if motorpara_lines is None:
            return False
        
        # 创建输出文件
        with open(output_file, 'w') as f:
            # 写入过滤后的固件内容（去掉结束记录）
            for line in firmware_lines:
                if not ':00000001FF' in line:
                    f.write(line + '\n')
            
            # 写入过滤后的电机参数内容
            for line in motorpara_lines:
                f.write(line + '\n')
        
        # 验证输出文件
        verify_hex_file(output_file)
        
        return True
        
    except FileNotFoundError as e:
        print(f"错误: 找不到输入文件 {e}")
        return False
    except Exception as e:
        print(f"合并HEX文件时出错: {e}")
        return False

def verify_hex_file(hex_file):
    """
    验证HEX文件，确保没有ADDRESS_THRESHOLD地址之后的数据
    """
    try:
        with open(hex_file, 'r') as f:
            lines = f.readlines()
        
        current_extended_address = 0x0000
        max_address = 0
        min_address = 0xFFFFFFFF
        has_eof = False
        data_record_count = 0
        problematic_records = []
        if DEBUG:
            print(f"验证HEX文件，检查是否包含0x{ADDRESS_THRESHOLD:08X}地址之后的数据...")
        
        for line in lines:
            line = line.strip()
            record = parse_hex_line(line)
            
            if record is None:
                continue
            
            # 更新扩展地址
            if record['record_type'] == 4:
                current_extended_address = int(record['data'][0:4], 16)
            
            # 如果是结束记录
            if record['record_type'] == 1:
                has_eof = True
                continue
            
            # 计算完整地址
            if record['record_type'] == 0:  # 数据记录
                data_record_count += 1
                full_address = (current_extended_address << 16) + record['address']
                
                # 更新最小和最大地址
                if full_address < min_address:
                    min_address = full_address
                if full_address > max_address:
                    max_address = full_address
                
                # 检查地址是否超过阈值
                if full_address > ADDRESS_THRESHOLD:
                    problematic_records.append({
                        'address': full_address,
                        'line': line
                    })
        if DEBUG:
            print(f"HEX文件统计:")
            print(f"  数据记录数量: {data_record_count}")
            print(f"  最小地址: 0x{min_address:08X}")
            print(f"  最大地址: 0x{max_address:08X}")
            print(f"  地址阈值: 0x{ADDRESS_THRESHOLD:08X}")
            print(f"  是否包含结束记录: {has_eof}")
        
        if problematic_records:
            print(f"错误: 发现 {len(problematic_records)} 条超过阈值的记录:")
            for record in problematic_records[:10]:  # 只显示前10条
                print(f"  地址: 0x{record['address']:08X}")
            if len(problematic_records) > 10:
                print(f"  ... 还有 {len(problematic_records) - 10} 条记录未显示")
            return False
        
        if max_address <= ADDRESS_THRESHOLD:
            if DEBUG:
                print(f"验证通过: 所有地址都在阈值范围内")
            return True
        else:
            print(f"警告: 最大地址0x{max_address:08X}超过阈值，但未发现具体问题记录")
            return True
        
    except Exception as e:
        print(f"验证HEX文件时出错: {e}")
        return False

def version_str_to_tuple(version_string):
    regex=r'.*v([0-9]+)\.([0-9]+)\.([0-9]+)(.*)'
    if not re.match(regex, version_string):
        raise Exception()
    return (int(re.sub(regex, r"\1", version_string)),
            int(re.sub(regex, r"\2", version_string)),
            int(re.sub(regex, r"\3", version_string)),
            (re.sub(regex, r"\4", version_string) != ""))

def get_version_from_git():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    try:
        # Determine the current git commit version
        git_tag = subprocess.check_output(["git", "describe", "--always", "--tags", "--dirty=*"],
            cwd=script_dir)
        git_tag = git_tag.decode(sys.stdout.encoding).rstrip('\n')

        (major, minor, revision, is_prerelease) = version_str_to_tuple(git_tag)

        # if is_prerelease:
        #     revision += 1
        return git_tag, major, minor, revision, is_prerelease

    except Exception as ex:
        print(ex)
        return "[unknown version]", 0, 0, 0, 1

def get_version_str(git_only=False, is_post_release=False, bump_rev=False, release_override=False):
    script_dir = os.path.dirname(os.path.realpath(__file__))

    # Try to read the version.txt file that is generated during
    # the packaging step
    version_file_path = os.path.join(script_dir, 'version.txt')
    if os.path.exists(version_file_path) and git_only == False:
        with open(version_file_path) as version_file:
            return version_file.readline().rstrip('\n')
    
    _, major, minor, revision, unreleased = get_version_from_git()
    if bump_rev:
        revision += 1
    version = '{}.{}.{}'.format(major, minor, revision)
    return version

def main():
    # 文件路径
    firmware_file = "../../Firmware/build/ODriveFirmware.hex"
    runmotorpara_file = "../../txt/光伏驱动轮/RunMotorPara.hex"
    brushmotorpara_file = "../../txt/光伏滚刷/BrushMotorPara.hex"

    # 检查输入文件是否存在
    if not os.path.exists(firmware_file):
        print(f"错误: 找不到固件文件 {firmware_file}")
        return 1
    
    if not os.path.exists(runmotorpara_file):
        print(f"错误: 找不到驱动轮电机参数文件 {runmotorpara_file}")
        return 1

    if not os.path.exists(brushmotorpara_file):
        print(f"错误: 找不到滚刷电机参数文件 {brushmotorpara_file}")
        return 1

    # 生成输出文件名
    version_str = get_version_str()
    run_motor_output_file = f"../../Firmware/build/Run_ODrive-fw-v{version_str}.hex"
    brush_motor_output_file = f"../../Firmware/build/Brush_ODrive-fw-v{version_str}.hex"
    if DEBUG:
        print(f"固件版本: {version_str}")
        print(f"地址阈值: 0x{ADDRESS_THRESHOLD:08X}")
        print("=" * 60)
    
    # 合并RunMotorHEX文件
    if DEBUG:
        print("开始处理驱动轮电机HEX文件...")
        print("-" * 60)
    
    if merge_hex_files(firmware_file, runmotorpara_file, run_motor_output_file):
        if DEBUG:
            print(f"驱动轮电机HEX文件生成成功: {run_motor_output_file}")
    else:
        print("驱动轮电机HEX文件生成失败！")
        return 1
    
    if DEBUG:
        print("\n" + "=" * 60)
    
    # 合并BrushMotorHEX文件
    if DEBUG:
        print("开始处理滚刷电机HEX文件...")
        print("-" * 60)
    
    if merge_hex_files(firmware_file, brushmotorpara_file, brush_motor_output_file):
        if DEBUG:
            print(f"滚刷电机HEX文件生成成功: {brush_motor_output_file}")
    else:
        print("滚刷电机HEX文件生成失败！")
        return 1
    
    if DEBUG:
        print("\n" + "=" * 60)
    print("固件合并完成！")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())