#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
协议解析测试脚本
模拟串口数据粘包情况并测试解析功能
"""

import time
import struct
from typing import List, Dict


class ProtocolTestParser:
    """协议测试解析器"""
    
    # 协议常量
    FRAME_HEADER = 0xAA
    PROTOCOL_VERSION = "V001"
    FIXED_LEN = 17  # 固定包头长度
    FIXED_WITHOUT_CRC_LEN = 15
    
    def __init__(self):
        self.buffer = bytearray()
        self.received_packets = []
        
    def crc16_modbus(self, data: bytes) -> int:
        """计算CRC16校验码"""
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
    
    def create_packet(self, cmd: int, data: bytes = None, msg_id: int = None) -> bytes:
        """创建协议数据包"""
        packet = bytearray()
        
        # 帧头
        packet.append(self.FRAME_HEADER)
        
        # 协议版本
        packet.extend(self.PROTOCOL_VERSION.encode('ascii'))
        
        # 数据长度
        data_len = len(data) if data else 0
        total_length = self.FIXED_LEN + data_len
        packet.extend(struct.pack('<H', total_length))
        
        # 消息ID
        if msg_id is None:
            msg_id = int(time.time() * 1000) & 0xFFFFFFFF
        packet.extend(struct.pack('<I', msg_id))
        
        # 命令
        packet.extend(struct.pack('<H', cmd))
        
        # 数据
        if data:
            packet.extend(data)
        
        # CRC校验
        crc = self.crc16_modbus(packet)
        packet.extend(struct.pack('<H', crc))
        
        return bytes(packet)
    
    def parse_packet(self, packet: bytes) -> Dict:
        """解析单个数据包"""
        if len(packet) < self.FIXED_LEN:
            return None
            
        # 验证帧头
        if packet[0] != self.FRAME_HEADER:
            return None
        
        # 解析各字段
        protocol_version = packet[1:5].decode('ascii', errors='ignore')
        length = struct.unpack('<H', packet[5:7])[0]
        msg_id = struct.unpack('<I', packet[7:11])[0]
        cmd = struct.unpack('<H', packet[11:13])[0]
        crc_received = struct.unpack('<H', packet[-2:])[0]
        
        # 验证CRC
        crc_calculated = self.crc16_modbus(packet[:-2])
        if crc_received != crc_calculated:
            return None
        
        # 提取数据
        data = packet[13:-2] if len(packet) > self.FIXED_LEN else b''
        
        return {
            'frame_header': packet[0],
            'protocol_version': protocol_version,
            'length': length,
            'msg_id': msg_id,
            'cmd': cmd,
            'crc': crc_received,
            'data': data,
            'raw_packet': packet
        }
    
    def process_data(self, new_data: bytes) -> List[Dict]:
        """处理新数据，处理粘包问题"""
        self.buffer.extend(new_data)
        packets = []
        
        print(f"接收新数据: {new_data.hex()}")
        print(f"当前缓冲区: {self.buffer.hex()}")
        
        while len(self.buffer) >= self.FIXED_LEN:
            # 查找帧头
            header_pos = -1
            for i in range(len(self.buffer)):
                if self.buffer[i] == self.FRAME_HEADER:
                    header_pos = i
                    break
            
            if header_pos == -1:
                print("未找到帧头，清空缓冲区")
                self.buffer.clear()
                break
            
            # 丢弃帧头前的数据
            if header_pos > 0:
                print(f"丢弃 {header_pos} 字节无效数据: {self.buffer[:header_pos].hex()}")
                self.buffer = self.buffer[header_pos:]
            
            # 检查是否有足够的数据读取长度字段
            if len(self.buffer) < 7:  # 帧头(1) + 协议版本(4) + 长度(2)
                print("数据不足，等待更多数据")
                break
            
            # 读取数据包长度
            length = struct.unpack('<H', self.buffer[5:7])[0]
            print(f"检测到数据包长度: {length}")
            
            # 检查是否有完整的数据包
            if len(self.buffer) < length:
                print(f"数据不完整，需要 {length} 字节，当前有 {len(self.buffer)} 字节")
                break
            
            # 提取并解析数据包
            packet = bytes(self.buffer[:length])
            parsed_packet = self.parse_packet(packet)
            
            if parsed_packet:
                packets.append(parsed_packet)
                print(f"成功解析数据包: {parsed_packet['cmd']:04X}")
            else:
                print("数据包解析失败，可能是损坏的数据")
            
            # 移除已处理的数据
            self.buffer = self.buffer[length:]
            print(f"剩余缓冲区: {self.buffer.hex()}")
        
        return packets
    
    def get_cmd_name(self, cmd: int) -> str:
        """获取命令名称"""
        cmd_names = {
            0x0102: "读取寄存器",
            0x0103: "上报寄存器", 
            0x0105: "写入寄存器",
            0x0106: "写入寄存器响应",
            0x00FF: "重启命令",
            0x0000: "响应命令",
            0x0001: "设备SN命令",
            0x0171: "软件版本命令"
        }
        return cmd_names.get(cmd, f"未知命令({cmd:04X})")
    
    def print_packet(self, packet: Dict):
        """打印数据包信息"""
        print("\n" + "="*50)
        print("解析成功的数据包:")
        print(f"  帧头: 0x{packet['frame_header']:02X}")
        print(f"  协议版本: {packet['protocol_version']}")
        print(f"  数据包长度: {packet['length']}")
        print(f"  消息ID: {packet['msg_id']}")
        print(f"  命令: 0x{packet['cmd']:04X} - {self.get_cmd_name(packet['cmd'])}")
        print(f"  CRC16: 0x{packet['crc']:04X}")
        
        if packet['data']:
            print(f"  数据长度: {len(packet['data'])}")
            print(f"  数据: {packet['data'].hex()}")
            # 尝试解析为ASCII
            try:
                ascii_data = packet['data'].decode('ascii')
                if ascii_data.isprintable():
                    print(f"  数据(ASCII): {ascii_data}")
            except:
                pass
        else:
            print("  数据: 无")
        print("="*50)


def test_sticky_packet_scenarios():
    """测试各种粘包场景"""
    parser = ProtocolTestParser()
    
    print("OriginFlow 协议粘包测试")
    print("="*60)
    
    # 场景1: 正常单个数据包
    print("\n场景1: 正常单个数据包")
    packet1 = parser.create_packet(0x0102, b'\x01\x00\x01\x00')
    parser.process_data(packet1)
    for packet in parser.received_packets[-1:]:
        parser.print_packet(packet)
    
    # 场景2: 两个完整数据包粘在一起
    print("\n场景2: 两个完整数据包粘在一起")
    packet2 = parser.create_packet(0x0103, b'\x02\x00\x02\x00')
    packet3 = parser.create_packet(0x00FF, b'')
    sticky_data = packet2 + packet3
    parser.process_data(sticky_data)
    for packet in parser.received_packets[-2:]:
        parser.print_packet(packet)
    
    # 场景3: 数据包分片传输
    print("\n场景3: 数据包分片传输")
    packet4 = parser.create_packet(0x0171, b'V1.0.0')
    # 分成3片传输
    chunk1 = packet4[:10]
    chunk2 = packet4[10:20]
    chunk3 = packet4[20:]
    
    parser.process_data(chunk1)
    parser.process_data(chunk2)
    parser.process_data(chunk3)
    for packet in parser.received_packets[-1:]:
        parser.print_packet(packet)
    
    # 场景4: 无效数据 + 有效数据包
    print("\n场景4: 无效数据 + 有效数据包")
    invalid_data = b'\x00\x01\x02\x03\xFF\xFF'
    packet5 = parser.create_packet(0x0001, b'SN123456')
    mixed_data = invalid_data + packet5
    parser.process_data(mixed_data)
    for packet in parser.received_packets[-1:]:
        parser.print_packet(packet)
    
    # 场景5: 不完整数据包 + 完整数据包
    print("\n场景5: 不完整数据包 + 完整数据包")
    packet6 = parser.create_packet(0x0105, b'\x01\x00\x00\x02')
    incomplete = packet6[:15]  # 只发送前15字节
    complete_rest = packet6[15:]  # 剩余部分
    
    parser.process_data(incomplete)
    parser.process_data(complete_rest)
    for packet in parser.received_packets[-1:]:
        parser.print_packet(packet)
    
    # 场景6: 多个数据包 + 无效数据混合
    print("\n场景6: 复杂混合场景")
    packets = [
        parser.create_packet(0x0000, b'OK'),
        parser.create_packet(0x0102, b'\x03\x00\x03\x00'),
        parser.create_packet(0x0106, b'SUCCESS')
    ]
    
    # 添加一些无效数据
    mixed_complex = b'\x55\xAA\xBB' + packets[0] + b'\x00\x11' + packets[1] + packets[2] + b'\xFF\xEE\xDD'
    
    parser.process_data(mixed_complex)
    for packet in parser.received_packets[-3:]:
        parser.print_packet(packet)


def interactive_test():
    """交互式测试"""
    parser = ProtocolTestParser()
    
    print("\n交互式协议测试")
    print("输入格式: <cmd_hex> [data_hex]")
    print("示例: 0102 01000100")
    print("输入 'quit' 退出")
    
    while True:
        try:
            user_input = input("\n> ").strip()
            
            if user_input.lower() == 'quit':
                break
            
            if not user_input:
                continue
            
            parts = user_input.split()
            cmd = int(parts[0], 16)
            data = bytes.fromhex(parts[1]) if len(parts) > 1 else b''
            
            # 创建数据包
            packet = parser.create_packet(cmd, data)
            print(f"创建数据包: {packet.hex()}")
            
            # 解析数据包
            parsed = parser.process_data(packet)
            for p in parsed:
                parser.print_packet(p)
                
        except ValueError as e:
            print(f"输入错误: {e}")
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    # 运行粘包测试场景
    test_sticky_packet_scenarios()
    
    # 运行交互式测试
    print("\n" + "="*60)
    interactive_test()
