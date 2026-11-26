#!/usr/bin/env python3
"""
ODrive固件拼接脚本
功能：
1. 将bootloader、主固件和多个参数文件分别合并,生成对应数量的新固件
2. 通过配置文件控制行为,配置文件示例(config.txt)
3. 从git tag中提取用户实际创建的tag名称
4. 生成新的HEX文件,命名为[参数文件名][tag名].hex
5. 移除指定地址之后的所有数据
6. 可选的清理输出目录功能
"""

import re
import os
import sys
import subprocess
import glob
import tempfile
from pathlib import Path

from extract_nvm_config import ihex_record

SHOW_LOGS = False

class Logger:
    """日志记录器，根据配置文件控制日志级别"""
    
    def __init__(self, show_debug=True, show_warning=True, show_error=True):
        self.show_debug = show_debug
        self.show_warning = show_warning
        self.show_error = show_error

    def debug(self, message):
        """调试信息"""
        if SHOW_LOGS:
            if self.show_debug:
                print(f"[DEBUG] {message}")
    
    def warning(self, message):
        """警告信息"""
        if self.show_warning:
            print(f"[WARNING] {message}")
    
    def error(self, message):
        """错误信息"""
        if self.show_error:
            print(f"[ERROR] {message}")
    
    def info(self, message):
        """普通信息（始终显示）"""
        if SHOW_LOGS:
            print(f"[INFO] {message}")
    
    def success(self, message):
        """成功信息"""
        print(f"[SUCCESS] {message}")

class ConfigParser:
    """配置文件解析器"""
    
    @staticmethod
    def parse_file_paths(paths_str):
        """
        解析逗号分隔的文件路径字符串
        """
        if not paths_str or not paths_str.strip():
            return []
        
        # 使用简单的分割方法
        paths = []
        for path in paths_str.split(','):
            path = path.strip().strip('"\'')
            if path:
                paths.append(path)
        
        return paths
    
    @staticmethod
    def parse_boolean(value):
        """解析布尔值"""
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            value_lower = value.lower()
            if value_lower in ['true', 'yes', '1', 'on']:
                return True
            elif value_lower in ['false', 'no', '0', 'off']:
                return False
        return True  # 默认值
    
    @staticmethod
    def parse_address_threshold(value):
        """
        解析地址阈值
        支持十六进制（0x开头）和十进制
        """
        if not value or not value.strip():
            # 默认值：0x08000000 - 1
            return 0x80FFFFF
        
        value = value.strip()
        
        # 十六进制
        if value.lower().startswith('0x'):
            try:
                return int(value, 16)
            except ValueError:
                print(f"[ERROR] 无法解析十六进制地址阈值 '{value}'，使用默认值 0x80FFFFF")
                return 0x80FFFFF
        
        # 十进制
        try:
            return int(value)
        except ValueError:
            print(f"[ERROR] 无法解析十进制地址阈值 '{value}'，使用默认值 0x80FFFFF")
            return 0x80FFFFF
    
    @staticmethod
    def parse_config(config_path, logger):
        """
        解析配置文件
        返回配置字典
        """
        config = {
            'should_continue': True,
            'show_debug': True,
            'show_warning': True,
            'show_error': True,
            'delete_hex': False,
            'address_threshold': 0x80FFFFF,
            'bootloader_file': "",
            'firmware_file': "",
            'datafile_paths': [],
            'released_mode': ""
        }
        
        if not os.path.exists(config_path):
            logger.error(f"配置文件 '{config_path}' 不存在")
            return config
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
        except Exception as e:
            logger.error(f"读取配置文件失败: {e}")
            return config
        
        for line in lines:
            # 跳过注释行和空行
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            # 解析键值对
            if '=' in line:
                key, value = line.split('=', 1)
                key = key.strip()
                value = value.strip()
                
                if key == 'should_continue':
                    config['should_continue'] = ConfigParser.parse_boolean(value)
                
                elif key == 'show_debug':
                    config['show_debug'] = ConfigParser.parse_boolean(value)
                
                elif key == 'show_warning':
                    config['show_warning'] = ConfigParser.parse_boolean(value)
                
                elif key == 'show_error':
                    config['show_error'] = ConfigParser.parse_boolean(value)
                
                elif key == 'delete_hex':
                    config['delete_hex'] = ConfigParser.parse_boolean(value)
                
                elif key == 'address_threshold':
                    config['address_threshold'] = ConfigParser.parse_address_threshold(value)
                
                elif key == 'firmware_file':
                    config['firmware_file'] = value.strip('"\'')

                elif key == 'bootloader_file':
                    config['bootloader_file'] = value.strip('"\'')
                
                elif key == 'datafile_paths':
                    config['datafile_paths'] = ConfigParser.parse_file_paths(value)
        
        return config

def get_git_tag_name(logger):
    """
    获取当前HEAD指向的git tag名称（用户实际创建的tag名）
    返回 (tag_name, is_dirty)
    """
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    try:
        # 方法1：使用git describe --exact-match获取精确的tag名
        # 这只在HEAD指向一个tag时有效
        git_tag = subprocess.check_output(
            ["git", "describe", "--exact-match", "--tags", "HEAD"],
            cwd=script_dir,
            stderr=subprocess.DEVNULL
        )
        tag_name = git_tag.decode(sys.stdout.encoding).rstrip('\n')
        logger.debug(f"获取到精确tag名称: {tag_name}")
        
        # 检查是否有未提交的修改
        try:
            git_status = subprocess.check_output(
                ["git", "status", "--porcelain"],
                cwd=script_dir,
                stderr=subprocess.DEVNULL
            )
            is_dirty = len(git_status.decode(sys.stdout.encoding).strip()) > 0
            logger.debug(f"是否有未提交修改: {is_dirty}")
        except:
            is_dirty = False
        
        return tag_name, is_dirty
        
    except subprocess.CalledProcessError as e:
        logger.warning(f"无法获取精确tag名称: {e}")
        
        # 方法2：如果不在tag上，尝试获取最近的tag
        try:
            git_tag = subprocess.check_output(
                ["git", "describe", "--tags", "--abbrev=0"],
                cwd=script_dir,
                stderr=subprocess.DEVNULL
            )
            tag_name = git_tag.decode(sys.stdout.encoding).rstrip('\n')
            logger.warning(f"使用最近tag: {tag_name}")
            
            # 检查是否有未提交的修改
            try:
                git_status = subprocess.check_output(
                    ["git", "status", "--porcelain"],
                    cwd=script_dir,
                    stderr=subprocess.DEVNULL
                )
                is_dirty = len(git_status.decode(sys.stdout.encoding).strip()) > 0
                logger.debug(f"是否有未提交修改: {is_dirty}")
            except:
                is_dirty = False
            
            return tag_name, is_dirty
            
        except subprocess.CalledProcessError as e2:
            logger.error(f"无法获取git tag名称: {e2}")
            return "v0.0.0", False
            
    except Exception as e:
        logger.error(f"获取git tag名称时出错: {e}")
        return "v0.0.0", False

def read_hex_file(path):
    """读取并验证 Intel HEX，返回绝对地址数据和可选的启动地址记录。"""
    data = {}
    base = 0
    start_record = None
    eof = False
    with open(path, 'r', encoding='ascii') as source:
        for line_number, line in enumerate(source, 1):
            line = line.strip()
            try:
                if eof or not line.startswith(':'):
                    raise ValueError("结束记录后有内容或缺少冒号")
                raw = bytes.fromhex(line[1:])
                if len(raw) < 5 or len(raw) != raw[0] + 5 or sum(raw) & 0xff:
                    raise ValueError("记录长度或校验和错误")
                count, address, record_type = raw[0], int.from_bytes(raw[1:3], 'big'), raw[3]
                payload = raw[4:-1]
                if record_type == 0:
                    if address + count > 0x10000 or base + address + count > 0x100000000:
                        raise ValueError("数据记录超出地址范围")
                    for offset, value in enumerate(payload):
                        absolute = base + address + offset
                        if absolute in data:
                            raise ValueError(f"地址 0x{absolute:08X} 重复")
                        data[absolute] = value
                elif record_type == 1 and count == 0 and address == 0:
                    eof = True
                elif record_type in (2, 4) and count == 2 and address == 0:
                    base = int.from_bytes(payload, 'big') << (4 if record_type == 2 else 16)
                elif record_type in (3, 5) and count == 4 and address == 0:
                    if start_record is not None:
                        raise ValueError("多个启动地址记录")
                    start_record = (record_type, payload)
                else:
                    raise ValueError(f"无效的记录类型或长度: {record_type}")
            except ValueError as exc:
                raise ValueError(f"{path}:{line_number}: {exc}") from exc
    if not eof:
        raise ValueError(f"{path}: 缺少结束记录")
    if not data:
        raise ValueError(f"{path}: 没有数据记录")
    return data, start_record


def merge_hex_files(bootloader_file, firmware_file, motorpara_file, output_file, address_threshold, logger):
    """按绝对地址合并三个HEX文件，保留bootloader的启动地址。"""
    try:
        merged = {}
        start_record = None
        for index, path in enumerate((bootloader_file, firmware_file, motorpara_file)):
            data, entry = read_hex_file(path)
            if index == 0:
                start_record = entry
            removed = sum(1 for address in data if address > address_threshold)
            if removed:
                logger.warning(f"{path}: 过滤了 {removed} 个超过地址阈值的字节")
            if removed == len(data):
                raise ValueError(f"{path}: 地址过滤后没有数据")
            for address, value in data.items():
                if address > address_threshold:
                    continue
                if address in merged:
                    raise ValueError(f"{path}: 地址 0x{address:08X} 与其他输入文件重叠")
                merged[address] = value

        if not merged:
            raise ValueError("地址过滤后没有数据")

        lines = []
        addresses = sorted(merged)
        position = 0
        upper = None
        while position < len(addresses):
            address = addresses[position]
            if address >> 16 != upper:
                upper = address >> 16
                lines.append(ihex_record(0, 4, upper.to_bytes(2, 'big')))
            chunk = bytearray()
            while (position < len(addresses) and len(chunk) < 16
                   and addresses[position] == address + len(chunk)
                   and addresses[position] >> 16 == upper):
                chunk.append(merged[addresses[position]])
                position += 1
            lines.append(ihex_record(address & 0xffff, 0, chunk))
        if start_record:
            lines.append(ihex_record(0, *start_record))
        lines.append(ihex_record(0, 1))

        output_dir = os.path.dirname(os.path.abspath(output_file))
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile('w', encoding='ascii', newline='\n',
                                             dir=output_dir, delete=False) as output:
                temp_path = output.name
                output.write('\n'.join(lines) + '\n')
            os.replace(temp_path, output_file)
        finally:
            if temp_path and os.path.exists(temp_path):
                os.remove(temp_path)
        return True
    except (OSError, UnicodeError, ValueError) as exc:
        logger.error(f"合并HEX文件时出错: {exc}")
        return False

def verify_hex_file(hex_file, address_threshold, logger):
    """
    验证HEX文件，确保没有超过指定地址之后的数据
    """
    try:
        data, _ = read_hex_file(hex_file)
        max_address = max(data)
        if max_address > address_threshold:
            logger.error(f"发现超过阈值的地址: 0x{max_address:08X}")
            return False
        logger.debug(f"验证通过: 最大地址 0x{max_address:08X} <= 阈值 0x{address_threshold:08X}")
        return True
    except (OSError, UnicodeError, ValueError) as e:
        logger.error(f"验证HEX文件时出错: {e}")
        return False

def generate_output_filename(datafile_path, git_tag, logger):
    """
    生成输出文件名
    格式: [参数文件名][tag名].hex
    示例: RunODrive-fw-v0.5.3.hex 或 RunODrive-fw-v0.5.3.beta1.hex
    """
    # 获取参数文件名（不含扩展名）
    datafile_name = Path(datafile_path).stem
    
    # 生成输出文件名
    # 格式: 参数文件名 + Git tag名 + .hex
    output_name = f"{datafile_name}{git_tag}.hex"
    
    logger.debug(f"生成输出文件名: {output_name}")
    return output_name

def check_file_exists(file_path, file_type="文件", logger=None):
    """
    检查文件是否存在
    """
    if not os.path.exists(file_path):
        if logger:
            logger.error(f"找不到{file_type}: {file_path}")
        else:
            print(f"[ERROR] 找不到{file_type}: {file_path}")
        return False
    return True

def check_hex_file(file_path, file_type="文件", logger=None):
    """
    检查文件是否为有效的HEX文件
    """
    try:
        with open(file_path, 'r') as f:
            first_line = f.readline().strip()
            if not first_line.startswith(':'):
                if logger:
                    logger.warning(f"{file_type}可能不是有效的HEX格式: {file_path}")
                else:
                    print(f"[WARNING] {file_type}可能不是有效的HEX格式: {file_path}")
                return False
        return True
    except Exception as e:
        if logger:
            logger.error(f"无法读取{file_type}: {file_path}, 错误: {e}")
        else:
            print(f"[ERROR] 无法读取{file_type}: {file_path}, 错误: {e}")
        return False

def clean_output_directory(output_dir, input_files, logger):
    """
    清理输出目录，保留所有输入HEX文件
    """
    try:
        if not os.path.exists(output_dir):
            logger.info(f"输出目录不存在，无需清理: {output_dir}")
            return 0
        
        # 获取所有HEX文件
        hex_files = glob.glob(os.path.join(output_dir, "*.hex"))
        deleted_count = 0
        
        for hex_file in hex_files:
            filename = os.path.basename(hex_file)
            
            # 跳过输入文件
            if os.path.normcase(os.path.abspath(hex_file)) in input_files:
                continue
            
            try:
                os.remove(hex_file)
                logger.info(f"删除文件: {filename}")
                deleted_count += 1
            except Exception as e:
                logger.error(f"无法删除文件 {filename}: {e}")
        
        if deleted_count > 0:
            logger.info(f"清理完成: 删除了 {deleted_count} 个HEX文件")
        else:
            logger.info("没有需要清理的HEX文件")
        
        return deleted_count
        
    except Exception as e:
        logger.error(f"清理输出目录时出错: {e}")
        return -1

def print_banner():
    """打印横幅"""
    banner = """
╔══════════════════════════════════════════════════════╗
║           ODrive固件拼接工具                         ║
║           ====================                       ║
║ 功能：                                              ║
║ 1. 合并主固件与参数文件                              ║
║ 2. 从Git获取精确的tag名称                            ║
║ 3. 生成命名规范的固件文件                            ║
║ 4. 移除超地址范围的数据                              ║
║ 5. 清理输出目录中的旧文件                            ║
╚══════════════════════════════════════════════════════╝
"""
    print(banner)

def main():
    """主函数"""
    # print_banner()
    
    # 配置文件路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(script_dir, "../../configFiles/版本说明文件/config.txt")
    
    # 首先读取配置以初始化日志器
    initial_config = ConfigParser.parse_config(config_file, Logger())
    
    # 创建日志器
    logger = Logger(
        show_debug=initial_config['show_debug'],
        show_warning=initial_config['show_warning'],
        show_error=initial_config['show_error']
    )
    logger.info("开始解析配置文件...")
    config = ConfigParser.parse_config(config_file, logger)
    for key in ('bootloader_file', 'firmware_file'):
        if config[key]:
            config[key] = os.path.abspath(os.path.join(script_dir, config[key]))
    config['datafile_paths'] = [
        os.path.abspath(os.path.join(script_dir, path)) for path in config['datafile_paths']
    ]
    
    # 显示配置信息
    logger.info("【配置信息】")
    logger.info(f"1. 是否继续执行: {config['should_continue']}")
    logger.info(f"2. 是否清理输出目录: {config['delete_hex']}")
    logger.info(f"3. 主固件文件: {config['firmware_file']}")
    logger.info(f"4. 参数文件数量: {len(config['datafile_paths'])}")
    
    for i, path in enumerate(config['datafile_paths'], 1):
        logger.info(f"   参数文件 {i}: {path}")
    
    logger.info(f"5. 地址阈值: 0x{config['address_threshold']:08X}")
    logger.info(f"6. 调试信息显示: {config['show_debug']}")
    logger.info(f"7. 警告信息显示: {config['show_warning']}")
    logger.info(f"8. 错误信息显示: {config['show_error']}")
    
    # 检查是否继续执行
    if not config['should_continue']:
        logger.info("根据配置文件设置，脚本将提前退出。")
        return 0
    
    if not config['bootloader_file']:
        logger.error("bootloader路径未指定")
        return 1
    if not check_file_exists(config['bootloader_file'], "bootloader文件", logger):
        return 1
    if not check_hex_file(config['bootloader_file'], "bootloader文件", logger):
        return 1

    # 验证主固件文件
    if not config['firmware_file']:
        logger.error("主固件路径未指定")
        return 1
    
    if not check_file_exists(config['firmware_file'], "主固件文件", logger):
        return 1
    
    if not check_hex_file(config['firmware_file'], "主固件文件", logger):
        return 1
    
    # 获取主固件文件名
    firmware_filename = os.path.basename(config['firmware_file'])
    
    # 清理输出目录（如果需要）
    if config['delete_hex']:
        logger.info("\n【清理输出目录】")
        output_dir = os.path.abspath(os.path.join(script_dir, "../../Firmware/build/"))
        input_files = {os.path.normcase(os.path.abspath(path)) for path in
                       [config['bootloader_file'], config['firmware_file']] + config['datafile_paths']}
        deleted_count = clean_output_directory(output_dir, input_files, logger)
        if deleted_count < 0:
            logger.error("清理输出目录失败")
            return 1
    
    # 获取git tag名称
    logger.info("\n【获取Git Tag信息】")
    git_tag, is_dirty = get_git_tag_name(logger)
    
    logger.info(f"Git Tag名称: {git_tag}")
    
    if is_dirty:
        logger.warning("警告: Git工作区有未提交的修改")
    
    # 处理每个参数文件
    logger.info(f"\n【开始处理参数文件】 (地址阈值: 0x{config['address_threshold']:08X})")
    successful_merges = 0
    total_files = len(config['datafile_paths'])
    
    for i, datafile_path in enumerate(config['datafile_paths'], 1):
        logger.info(f"\n{'━' * 50}")
        logger.info(f"处理参数文件 [{i}/{total_files}]")
        logger.info(f"{'━' * 50}")
        logger.info(f"参数文件: {datafile_path}")
        
        # 验证参数文件
        if not check_file_exists(datafile_path, "参数文件", logger):
            continue
        
        if not check_hex_file(datafile_path, "参数文件", logger):
            continue
        
        # 获取参数文件名（不含扩展名）
        param_name = Path(datafile_path).stem
        logger.info(f"参数名称: {param_name}")
        
        # 生成输出文件名
        output_filename = f"{param_name}{git_tag}.hex"
        logger.info(f"输出文件名: {output_filename}")
        
        # 创建输出目录
        output_dir = os.path.abspath(os.path.join(script_dir, "../../Firmware/build/"))
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, output_filename)
        
        input_files = [config['bootloader_file'], config['firmware_file']] + config['datafile_paths']
        if os.path.normcase(os.path.abspath(output_path)) in {
                os.path.normcase(os.path.abspath(path)) for path in input_files}:
            logger.error(f"输出文件不能覆盖输入文件: {output_path}")
            continue
        
        # 合并文件
        logger.info("开始合并文件...")
        if merge_hex_files(config['bootloader_file'], config['firmware_file'], datafile_path,
                           output_path, config['address_threshold'], logger):
            # 验证输出文件
            logger.info("验证输出文件...")
            if verify_hex_file(output_path, config['address_threshold'], logger):
                logger.success(f"合并成功: {output_filename}")
                
                # 显示文件信息
                try:
                    file_size = os.path.getsize(output_path)
                    logger.info(f"  文件大小: {file_size:,} 字节")
                    logger.info(f"  输出路径: {output_path}")
                except Exception as e:
                    logger.warning(f"无法获取文件大小: {e}")
                
                successful_merges += 1
            else:
                logger.error("输出文件验证失败")
                # 删除无效文件
                try:
                    if os.path.exists(output_path):
                        os.remove(output_path)
                except Exception as e:
                    logger.error(f"无法删除无效文件: {e}")
        else:
            logger.error("文件合并失败")
    
    # 处理结果总结
    logger.info(f"\n{'═' * 60}")
    logger.info("【处理完成】")
    logger.info(f"成功合并: {successful_merges}/{total_files} 个文件")
    
    if successful_merges > 0:
        output_dir = os.path.abspath(os.path.join(script_dir, "../../Firmware/build/"))
        logger.info(f"\n生成的文件保存在: {os.path.abspath(output_dir)}")
        
        # 显示生成的输出文件列表
        logger.info("生成的文件列表:")
        if os.path.exists(output_dir):
            hex_files = []
            for file in sorted(os.listdir(output_dir)):
                if file.endswith('.hex'):
                    filepath = os.path.join(output_dir, file)
                    try:
                        size = os.path.getsize(filepath)
                        hex_files.append((file, size))
                    except:
                        hex_files.append((file, 0))
            
            if hex_files:
                for file, size in hex_files:
                    if file == firmware_filename:
                        logger.info(f"  📁 {file} (主固件文件)")
                    elif size > 0:
                        logger.info(f"  ✓ {file} ({size:,} 字节)")
                    else:
                        logger.info(f"  ✓ {file}")
            else:
                logger.warning("  未找到HEX文件")
    else:
        logger.error("没有成功生成任何文件")
    
    logger.info(f"{'═' * 60}")
    
    return 0 if total_files > 0 and successful_merges == total_files else 1

if __name__ == "__main__":
    sys.exit(main())