#!/usr/bin/env python3
"""
ODrive固件拼接脚本
功能：
1. 读取ODriveFirmware.hex和MotorPara.hex文件
2. 从version.c文件中提取版本号
3. 生成新的HEX文件,t命名为ODrive-fw-v{version}.hex
"""

import re
import os
import sys

def parse_version_from_c_file(file_path):
    """
    从version.c文件中解析版本号
    格式示例：
    const unsigned char fw_version_major_ = 0;
    const unsigned char fw_version_minor_ = 5;
    const unsigned char fw_version_revision_ = 3;
    const unsigned char fw_version_unreleased_ = 1;
    
    返回格式:0.5.3.1
    """
    version_major = 0
    version_minor = 0
    version_revision = 0
    version_unreleased = 0
    
    try:
        with open(file_path, 'r') as f:
            content = f.read()
            
            # 使用正则表达式匹配版本号
            major_match = re.search(r'fw_version_major_\s*=\s*(\d+)', content)
            minor_match = re.search(r'fw_version_minor_\s*=\s*(\d+)', content)
            revision_match = re.search(r'fw_version_revision_\s*=\s*(\d+)', content)
            unreleased_match = re.search(r'fw_version_unreleased_\s*=\s*(\d+)', content)
            
            if major_match:
                version_major = int(major_match.group(1))
            if minor_match:
                version_minor = int(minor_match.group(1))
            if revision_match:
                version_revision = int(revision_match.group(1))
            if unreleased_match:
                version_unreleased = int(unreleased_match.group(1))
                
        return f"{version_major}.{version_minor}.{version_revision}.{version_unreleased}"
        
    except FileNotFoundError:
        print(f"错误: 找不到文件 {file_path}")
        return None
    except Exception as e:
        print(f"解析version.c文件时出错: {e}")
        return None

def merge_hex_files(firmware_file, motorpara_file, output_file):
    """
    合并两个HEX文件
    """
    try:
        # 读取固件文件
        with open(firmware_file, 'r') as f:
            firmware_lines = f.readlines()
        
        # 读取电机参数文件
        with open(motorpara_file, 'r') as f:
            motorpara_lines = f.readlines()
        
        # 确保有结束记录
        firmware_has_eof = any(':00000001FF' in line for line in firmware_lines)
        
        # 创建输出文件
        with open(output_file, 'w') as f:
            # 写入固件内容（去掉结束记录）
            for line in firmware_lines:
                if not ':00000001FF' in line:
                    f.write(line)
            
            # 写入电机参数内容
            for line in motorpara_lines:
                f.write(line)
        
        print(f"成功合并HEX文件: {output_file}")
        return True
        
    except FileNotFoundError as e:
        print(f"错误: 找不到输入文件 {e}")
        return False
    except Exception as e:
        print(f"合并HEX文件时出错: {e}")
        return False

def main():
    # 文件路径
    firmware_file = "./build/ODriveFirmware.hex"
    motorpara_file = "../txt/光伏驱动轮/MotorPara.hex"
    version_file = "./autogen/version.c"
    
    # 检查输入文件是否存在
    if not os.path.exists(firmware_file):
        print(f"错误: 找不到固件文件 {firmware_file}")
        return 1
    
    if not os.path.exists(motorpara_file):
        print(f"错误: 找不到电机参数文件 {motorpara_file}")
        return 1
    
    # 解析版本号
    version = parse_version_from_c_file(version_file)
    if version is None:
        print("警告: 无法解析版本号，使用默认版本 0.0.0.0")
        version = "0.0.0.0"
    else:
        print(f"成功解析版本号: v{version}")
    
    # 生成输出文件名
    output_file = f"./build/ODrive-fw-v{version}.hex"
    
    # 检查输出文件是否已存在
    if os.path.exists(output_file):
        overwrite = input(f"文件 {output_file} 已存在，是否覆盖？(y/n): ")
        if overwrite.lower() != 'y':
            print("操作已取消")
            return 0
    
    # 合并HEX文件
    print(f"开始合并HEX文件...")
    print(f"固件文件: {firmware_file}")
    print(f"参数文件: {motorpara_file}")
    print(f"输出文件: {output_file}")
    
    if merge_hex_files(firmware_file, motorpara_file, output_file):
        # 验证输出文件
        if os.path.exists(output_file):
            with open(output_file, 'r') as f:
                lines = f.readlines()
                print(f"生成的HEX文件包含 {len(lines)} 行")
                
                # 检查是否有结束记录
                has_eof = any(':00000001FF' in line for line in lines)
                if not has_eof:
                    print("警告: 生成的HEX文件没有结束记录")
                else:
                    print("✓ HEX文件包含正确的结束记录")
        
        print("\n操作完成！")
        print(f"生成的文件: {output_file}")
        return 0
    else:
        return 1

if __name__ == "__main__":
    sys.exit(main())