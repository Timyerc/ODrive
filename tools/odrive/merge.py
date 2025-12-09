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
import subprocess
import platform

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
        
        #print(f"成功合并HEX文件: {output_file}")
        return True
        
    except FileNotFoundError as e:
        print(f"错误: 找不到输入文件 {e}")
        return False
    except Exception as e:
        print(f"合并HEX文件时出错: {e}")
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
    if is_post_release:
        version += ".post"
    elif not release_override and unreleased:
        version += ".dev"
    return version

def main():
    # 文件路径
    firmware_file = "../../Firmware/build/ODriveFirmware.hex"
    runmotorpara_file = "../../txt/光伏驱动轮/RunMotorPara.hex"
    brushmotorpara_file = "../../txt/光伏滚刷/BrushMotorPara.hex"

    # 新文件名及版本号
    """
    git_name, major, minor, revision, unreleased = get_version_from_git()
    print('Firmware version {}.{}.{}{} ({})'.format(
        major, minor, revision, '-dev' if unreleased else '',
        git_name))
    """

    
    # 检查输入文件是否存在
    if not os.path.exists(firmware_file):
        print(f"错误: 找不到固件文件 {firmware_file}")
        return 1
    
    if not os.path.exists(runmotorpara_file):
        print(f"错误: 找不到驱动轮电机参数文件 {runmotorpara_file}")
        return 1

    if not os.path.exists(brushmotorpara_file):
        print(f"错误: 找不到滚刷电机参数文件 {runmotorpara_file}")
        return 1

    # 生成输出文件名
    run_motor_output_file = f"../../Firmware/build/Run_ODrive-fw-v{get_version_str()}.hex"
    brush_motor_output_file = f"../../Firmware/build/Brush_ODrive-fw-v{get_version_str()}.hex"

    # 合并RunMotorHEX文件
    if merge_hex_files(firmware_file, runmotorpara_file, run_motor_output_file):
        # 验证输出文件
        if os.path.exists(run_motor_output_file):
            with open(run_motor_output_file, 'r') as f:
                lines = f.readlines()
                print(f"生成的驱动轮电机HEX文件包含 {len(lines)} 行")
                
                # 检查是否有结束记录
                has_eof = any(':00000001FF' in line for line in lines)
                if not has_eof:
                    print("警告: 生成的驱动轮电机HEX文件没有结束记录")
                else:
                    print("驱动轮电机HEX文件包含正确的结束记录")
        
        
        print(f"输出文件路径: {run_motor_output_file}")
        print("操作完成！")
    # 合并BrushMotorHEX文件
    if merge_hex_files(firmware_file, brushmotorpara_file, brush_motor_output_file):
        # 验证输出文件
        if os.path.exists(brush_motor_output_file):
            with open(brush_motor_output_file, 'r') as f:
                lines = f.readlines()
                print(f"生成滚刷电机HEX文件包含 {len(lines)} 行")
                
                # 检查是否有结束记录
                has_eof = any(':00000001FF' in line for line in lines)
                if not has_eof:
                    print("警告: 生成的滚刷电机HEX文件没有结束记录")
                else:
                    print("滚刷电机HEX文件包含正确的结束记录")
        
        
        print(f"输出文件路径: {brush_motor_output_file}")
        print("操作完成！")

if __name__ == "__main__":
    sys.exit(main())