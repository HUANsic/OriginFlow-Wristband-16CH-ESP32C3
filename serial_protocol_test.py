#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口协议测试工具
支持读取串口数据并解析协议处理粘包问题
"""

import serial
import serial.tools.list_ports
import time
import threading
import struct
from collections import deque
from typing import Optional, Tuple, List


class ProtocolParser:
    """协议解析器类"""
    
    # 协议常量定义
    FRAME_HEADER = 0xAA
    FRAME_HEADER_LENGTH = 1
    PROTOCOL_VERSION = "V001"
    PROTOCOL_VERSION_LENGTH = 4
    MSG_ID_LENGTH = 4
    LEN_LENGTH = 2
    
    # 固定长度计算 (6 + LEN_LENGTH + FRAMEHEADER_LENGTH + MSG_ID_LENGTH + PROTOCOL_VERSION_LENGTH)
    FIXED_LEN = 6 + 2 + 1 + 4 + 4  # 17字节
    FIXED_WITHOUT_CRC_LEN = 4 + 2 + 1 + 4 + 4  # 15字节
    
    def __init__(self):
        self.buffer = bytearray()
        self.packets = deque()
        
    def crc16_modbus(self, data: bytes) -> int:
        """计算CRC16校验码 (Modbus)"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
    
    def parse_packet(self, packet: bytes) -> dict:
        """解析单个数据包"""
        if len(packet) < self.FIXED_LEN:
            return None
            
        # 解析固定头部
        offset = 0
        
        # 帧头
        frame_header = packet[offset]
        offset += self.FRAME_HEADER_LENGTH
        
        # 协议版本
        protocol_version = packet[offset:offset+self.PROTOCOL_VERSION_LENGTH].decode('ascii', errors='ignore')
        offset += self.PROTOCOL_VERSION_LENGTH
        
        # 长度
        length = struct.unpack('<H', packet[offset:offset+self.LEN_LENGTH])[0]
        offset += self.LEN_LENGTH
        
        # 消息ID
        msg_id = struct.unpack('<I', packet[offset:offset+self.MSG_ID_LENGTH])[0]
        offset += self.MSG_ID_LENGTH
        
        # 命令
        cmd = struct.unpack('<H', packet[offset:offset+2])[0]
        offset += 2
        
        # CRC16
        crc_received = struct.unpack('<H', packet[offset:offset+2])[0]
        offset += 2
        
        # 验证CRC
        crc_calculated = self.crc16_modbus(packet[:-2])
        if crc_received != crc_calculated:
            return None
        
        # 提取数据部分
        data = packet[offset:] if len(packet) > offset else b''
        
        return {
            'frame_header': frame_header,
            'protocol_version': protocol_version,
            'length': length,
            'msg_id': msg_id,
            'cmd': cmd,
            'crc': crc_received,
            'data': data,
            'raw_packet': packet
        }
    
    def find_frame_header(self, data: bytes) -> int:
        """查找帧头位置"""
        for i, byte in enumerate(data):
            if byte == self.FRAME_HEADER:
                return i
        return -1
    
    def process_data(self, new_data: bytes) -> List[dict]:
        """处理新接收的数据，返回完整的数据包列表"""
        self.buffer.extend(new_data)
        packets = []
        
        while len(self.buffer) >= self.FIXED_LEN:
            # 查找帧头
            header_pos = self.find_frame_header(self.buffer)
            if header_pos == -1:
                # 没有找到帧头，清空缓冲区
                self.buffer.clear()
                break
            
            # 如果帧头不在开始位置，丢弃前面的数据
            if header_pos > 0:
                self.buffer = self.buffer[header_pos:]
            
            # 检查是否有足够的数据读取长度字段
            if len(self.buffer) < self.FRAME_HEADER_LENGTH + self.PROTOCOL_VERSION_LENGTH + self.LEN_LENGTH:
                break
            
            # 读取数据包长度
            length_offset = self.FRAME_HEADER_LENGTH + self.PROTOCOL_VERSION_LENGTH
            packet_length = struct.unpack('<H', self.buffer[length_offset:length_offset+self.LEN_LENGTH])[0]
            
            # 检查是否有完整的数据包
            if len(self.buffer) < packet_length:
                break
            
            # 提取完整数据包
            packet = self.buffer[:packet_length]
            
            # 解析数据包
            parsed_packet = self.parse_packet(packet)
            if parsed_packet:
                packets.append(parsed_packet)
            
            # 移除已处理的数据包
            self.buffer = self.buffer[packet_length:]
        
        return packets


class SerialProtocolTester:
    """串口协议测试器"""
    
    def __init__(self, port: str = None, baudrate: int = 115200):
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.parser = ProtocolParser()
        self.running = False
        self.read_thread = None
        
    def list_ports(self) -> List[str]:
        """列出可用的串口"""
        ports = []
        for port_info in serial.tools.list_ports.comports():
            ports.append(f"{port_info.device} - {port_info.description}")
        return ports
    
    def connect(self, port: str = None, baudrate: int = None) -> bool:
        """连接串口"""
        if port:
            self.port = port
        if baudrate:
            self.baudrate = baudrate
            
        if not self.port:
            print("错误: 未指定串口")
            return False
            
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"成功连接到串口: {self.port} ({self.baudrate} bps)")
            return True
        except Exception as e:
            print(f"连接串口失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        self.running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join()
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("串口已断开")
    
    def start_reading(self):
        """开始读取数据"""
        if not self.serial_port or not self.serial_port.is_open:
            print("错误: 串口未连接")
            return
            
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        print("开始读取串口数据...")
    
    def _read_loop(self):
        """读取数据循环"""
        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    packets = self.parser.process_data(data)
                    
                    for packet in packets:
                        self._packet_received(packet)
                        
                time.sleep(0.001)  # 短暂休眠避免CPU占用过高
                
            except Exception as e:
                print(f"读取数据错误: {e}")
                break
    
    def _packet_received(self, packet: dict):
        """处理接收到的数据包"""
        print("\n" + "="*60)
        print("收到数据包:")
        print(f"  帧头: 0x{packet['frame_header']:02X}")
        print(f"  协议版本: {packet['protocol_version']}")
        print(f"  数据包长度: {packet['length']}")
        print(f"  消息ID: {packet['msg_id']}")
        print(f"  命令: 0x{packet['cmd']:04X} ({self._get_cmd_name(packet['cmd'])})")
        print(f"  CRC16: 0x{packet['crc']:04X}")
        
        if packet['data']:
            print(f"  数据: {packet['data'].hex()}")
            print(f"  数据(ASCII): {packet['data'].decode('ascii', errors='ignore')}")
        else:
            print("  数据: 无")
            
        print(f"  原始数据包: {packet['raw_packet'].hex()}")
        print("="*60)
    
    def _get_cmd_name(self, cmd: int) -> str:
        """获取命令名称"""
        cmd_names = {
            0x0000: "响应命令",
            0x0001: "设备SN命令",
            0x0002: "设备密钥命令",
            0x0003: "设备协议版本命令",
            0x0004: "IP命令",
            0x0005: "查找客户端/服务器命令",
            0x0006: "BLE命令",
            0x0008: "连接设备命令",
            0x0009: "断开连接命令",
            0x000A: "WiFi命令",
            0x000B: "设备连接状态命令",
            0x00FB: "测试命令",
            0x00FD: "恢复出厂设置命令",
            0x00FE: "关机命令",
            0x00FF: "重启命令",
            0x0100: "读取多寄存器命令",
            0x0101: "上报多寄存器命令",
            0x0102: "读取寄存器命令",
            0x0103: "上报寄存器命令",
            0x0104: "写入多寄存器命令",
            0x0105: "写入寄存器命令",
            0x0106: "写入多寄存器响应命令",
            0x0107: "写入寄存器响应命令",
            0x0170: "文件更新命令",
            0x0171: "软件版本命令",
            0x0172: "发送文件命令",
            0x0173: "HTTP OTA更新命令",
            0x0175: "获取最新版本命令",
            0xFFFE: "信息命令",
            0xFFFF: "错误命令"
        }
        return cmd_names.get(cmd, f"未知命令(0x{cmd:04X})")
    
    def send_test_packet(self, cmd: int, data: bytes = None):
        """发送测试数据包"""
        if not self.serial_port or not self.serial_port.is_open:
            print("错误: 串口未连接")
            return
            
        # 构建数据包
        packet = bytearray()
        
        # 帧头
        packet.append(self.FRAME_HEADER)
        
        # 协议版本
        packet.extend(self.PROTOCOL_VERSION.encode('ascii'))
        
        # 数据长度 (固定长度 + 数据长度)
        data_len = len(data) if data else 0
        total_length = self.FIXED_LEN + data_len
        packet.extend(struct.pack('<H', total_length))
        
        # 消息ID (使用时间戳的低32位)
        msg_id = int(time.time()) & 0xFFFFFFFF
        packet.extend(struct.pack('<I', msg_id))
        
        # 命令
        packet.extend(struct.pack('<H', cmd))
        
        # 数据
        if data:
            packet.extend(data)
        
        # 计算并添加CRC
        crc = self.parser.crc16_modbus(packet)
        packet.extend(struct.pack('<H', crc))
        
        # 发送数据包
        try:
            self.serial_port.write(packet)
            print(f"发送测试数据包: {packet.hex()}")
            print(f"  命令: 0x{cmd:04X} ({self._get_cmd_name(cmd)})")
            if data:
                print(f"  数据: {data.hex()}")
        except Exception as e:
            print(f"发送数据包失败: {e}")


def main():
    """主函数"""
    print("OriginFlow 手环协议测试工具")
    print("="*50)
    
    tester = SerialProtocolTester()
    
    # 列出可用串口
    print("可用串口:")
    ports = tester.list_ports()
    for i, port in enumerate(ports):
        print(f"  {i+1}. {port}")
    
    if not ports:
        print("未找到可用串口")
        return
    
    # 选择串口
    try:
        port_index = int(input("\n请选择串口 (输入序号): ")) - 1
        if port_index < 0 or port_index >= len(ports):
            print("无效的串口序号")
            return
        
        selected_port = ports[port_index].split(' - ')[0]
        
        # 选择波特率
        baudrate = input("请输入波特率 (默认115200): ").strip()
        baudrate = int(baudrate) if baudrate else 115200
        
    except ValueError:
        print("输入无效")
        return
    
    # 连接串口
    if not tester.connect(selected_port, baudrate):
        return
    
    # 开始读取数据
    tester.start_reading()
    
    print("\n命令说明:")
    print("  send <cmd> <data>  - 发送测试数据包 (cmd为16进制，data为16进制字符串)")
    print("  quit               - 退出程序")
    print("  help               - 显示帮助信息")
    print("\n常用命令:")
    print("  send 0x0102 0x0101 - 发送读取寄存器命令")
    print("  send 0x0105 0x01010002 - 发送写入寄存器命令")
    print("  send 0x00FF         - 发送重启命令")
    
    try:
        while True:
            cmd_input = input("\n> ").strip()
            
            if cmd_input.lower() == 'quit':
                break
            elif cmd_input.lower() == 'help':
                print("命令说明:")
                print("  send <cmd> <data>  - 发送测试数据包")
                print("  quit               - 退出程序")
                continue
            elif cmd_input.startswith('send '):
                parts = cmd_input.split()
                if len(parts) >= 2:
                    try:
                        cmd = int(parts[1], 16)
                        data = bytes.fromhex(parts[2]) if len(parts) > 2 else None
                        tester.send_test_packet(cmd, data)
                    except ValueError:
                        print("无效的16进制数值")
                else:
                    print("用法: send <cmd> [data]")
            else:
                print("未知命令，输入 'help' 查看帮助")
                
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        tester.disconnect()


if __name__ == "__main__":
    main()
