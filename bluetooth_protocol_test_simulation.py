#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
蓝牙GATT粘包测试脚本
模拟各种蓝牙GATT数据传输场景并测试协议解析
"""

import asyncio
import struct
import time
from typing import List, Dict

# 协议常量
FRAME_HEADER = 0xAA
PROTOCOL_VERSION = "V001"
FIXED_LEN = 17
FIXED_WITHOUT_CRC_LEN = 15

# GATT服务信息
SERVICE_UUID = "FB349B5F-8000-0080-0010-0000FF0001"
CHARACTERISTIC_UUID = "FB349B5F-8000-0080-0010-0000FF0002"


class BluetoothProtocolTestParser:
    """蓝牙协议测试解析器"""
    
    def __init__(self):
        self.buffer = bytearray()
        self.received_packets = []
        self.total_received = 0
        
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
        packet.append(FRAME_HEADER)
        
        # 协议版本
        packet.extend(PROTOCOL_VERSION.encode('ascii'))
        
        # 数据长度
        data_len = len(data) if data else 0
        total_length = FIXED_LEN + data_len
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
        if len(packet) < FIXED_LEN:
            return None
            
        # 验证帧头
        if packet[0] != FRAME_HEADER:
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
        data = packet[13:-2] if len(packet) > FIXED_LEN else b''
        
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
    
    def process_gatt_data(self, new_data: bytes) -> List[Dict]:
        """处理GATT接收的数据，模拟粘包情况"""
        self.buffer.extend(new_data)
        self.total_received += len(new_data)
        packets = []
        
        print(f"\nGATT接收数据: {new_data.hex()}")
        print(f"当前缓冲区: {self.buffer.hex()}")
        print(f"累计接收字节: {self.total_received}")
        
        while len(self.buffer) >= FIXED_LEN:
            # 查找帧头
            header_pos = -1
            for i in range(len(self.buffer)):
                if self.buffer[i] == FRAME_HEADER:
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
                print(f"✓ 成功解析数据包: 命令=0x{parsed_packet['cmd']:04X}")
            else:
                print("✗ 数据包解析失败，可能是损坏的数据")
            
            # 移除已处理的数据
            self.buffer = self.buffer[length:]
            print(f"剩余缓冲区: {self.buffer.hex()}")
        
        return packets
    
    def get_cmd_name(self, cmd: int) -> str:
        """获取命令名称"""
        cmd_names = {
            0x0000: "响应命令",
            0x0001: "设备SN命令",
            0x0002: "设备密钥命令",
            0x0003: "设备协议版本命令",
            0x0102: "读取寄存器",
            0x0103: "上报寄存器",
            0x0105: "写入寄存器",
            0x0106: "写入寄存器响应",
            0x00FF: "重启命令",
            0x0171: "软件版本命令",
            0x00FB: "测试命令"
        }
        return cmd_names.get(cmd, f"未知命令({cmd:04X})")
    
    def print_packet(self, packet: Dict):
        """打印数据包信息"""
        print("\n" + "="*60)
        print("🔵 蓝牙GATT数据包解析成功:")
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
        print("="*60)
    
    def print_gatt_info(self):
        """打印GATT服务信息"""
        print("\n" + "="*60)
        print("📡 OriginFlow 蓝牙GATT服务信息:")
        print(f"  服务UUID: {SERVICE_UUID}")
        print(f"  特征UUID: {CHARACTERISTIC_UUID}")
        print("  支持操作: NOTIFY, WRITE")
        print("  协议格式: OriginFlow 自定义协议")
        print("="*60)


def test_bluetooth_gatt_scenarios():
    """测试各种蓝牙GATT粘包场景"""
    parser = BluetoothProtocolTestParser()
    
    print("OriginFlow 蓝牙GATT协议粘包测试")
    parser.print_gatt_info()
    
    # 场景1: 正常单个数据包
    print("\n🔹 场景1: 正常单个GATT数据包")
    packet1 = parser.create_packet(0x0103, b'\x01\x00\x01\x00')  # 上报寄存器
    parser.process_gatt_data(packet1)
    for packet in parser.received_packets[-1:]:
        parser.print_packet(packet)
    
    # 场景2: 多个数据包在一次GATT通知中
    print("\n🔹 场景2: 多个数据包在一次GATT通知中 (常见粘包)")
    packet2 = parser.create_packet(0x0103, b'\x02\x00\x02\x00')
    packet3 = parser.create_packet(0x0000, b'OK')  # 响应
    packet4 = parser.create_packet(0x0171, b'V1.0.0')  # 软件版本
    combined_data = packet2 + packet3 + packet4
    parser.process_gatt_data(combined_data)
    for packet in parser.received_packets[-3:]:
        parser.print_packet(packet)
    
    # 场景3: GATT MTU限制导致的数据分片
    print("\n🔹 场景3: GATT MTU限制导致的数据分片")
    large_data = b'A' * 50  # 50字节数据
    packet5 = parser.create_packet(0x0103, large_data)
    
    # 模拟MTU=20，分片传输
    mtu_size = 20
    for i in range(0, len(packet5), mtu_size):
        chunk = packet5[i:i+mtu_size]
        print(f"--- GATT分片 {i//mtu_size + 1} ---")
        parser.process_gatt_data(chunk)
    
    for packet in parser.received_packets[-1:]:
        parser.print_packet(packet)
    
    # 场景4: GATT通知丢失和重复
    print("\n🔹 场景4: GATT通知丢失和重复")
    packet6 = parser.create_packet(0x0105, b'\x01\x00\x00\x02')  # 写入寄存器
    packet7 = parser.create_packet(0x0106, b'SUCCESS')  # 写入响应
    
    # 模拟通知丢失 (只发送packet6)
    parser.process_gatt_data(packet6)
    
    # 模拟通知重复 (重复发送packet7)
    parser.process_gatt_data(packet7)
    parser.process_gatt_data(packet7)  # 重复
    
    for packet in parser.received_packets[-2:]:
        parser.print_packet(packet)
    
    # 场景5: 无效数据干扰
    print("\n🔹 场景5: 无效数据干扰")
    invalid_data = b'\x00\x55\xFF\xEE\xCC\xBB'
    packet8 = parser.create_packet(0x00FB, b'TEST_DATA')  # 测试命令
    mixed_data = invalid_data + packet8 + invalid_data
    parser.process_gatt_data(mixed_data)
    for packet in parser.received_packets[-1:]:
        parser.print_packet(packet)
    
    # 场景6: 不完整数据包
    print("\n🔹 场景6: 不完整数据包")
    packet9 = parser.create_packet(0x0001, b'SN123456789')
    
    # 分成不完整的几块
    chunk1 = packet9[:10]      # 不完整
    chunk2 = packet9[10:15]   # 不完整  
    chunk3 = packet9[15:]     # 剩余部分
    
    parser.process_gatt_data(chunk1)
    parser.process_gatt_data(chunk2)
    parser.process_gatt_data(chunk3)
    
    for packet in parser.received_packets[-1:]:
        parser.print_packet(packet)
    
    # 场景7: 复杂混合场景
    print("\n🔹 场景7: 复杂混合场景 (模拟真实环境)")
    
    # 创建多个不同大小的数据包
    packets = [
        parser.create_packet(0x0103, b'\x01'),                    # 小数据包
        parser.create_packet(0x0103, b'\x02\x00\x02\x00'),         # 中等数据包
        parser.create_packet(0x0171, b'V2.1.0-Beta'),             # 带ASCII的数据包
        parser.create_packet(0x0000, b'OK'),                      # 响应包
        parser.create_packet(0x00FF, b''),                         # 无数据包
    ]
    
    # 模拟真实的GATT传输：随机组合、分片、干扰
    import random
    
    # 随机打乱
    random.shuffle(packets)
    
    # 添加一些干扰数据
    interference = b'\xAA\xBB\xCC\xDD'
    
    # 创建复杂的数据流
    complex_stream = interference + packets[0] + packets[1][:10]
    complex_stream += packets[1][10:] + packets[2] + interference
    complex_stream += packets[3] + packets[4] + packets[0]  # 重复一个包
    
    print(f"复杂数据流长度: {len(complex_stream)} 字节")
    parser.process_gatt_data(complex_stream)
    
    print(f"\n总计解析出 {len(parser.received_packets)} 个有效数据包")


def simulate_gatt_notification_stream():
    """模拟连续的GATT通知流"""
    parser = BluetoothProtocolTestParser()
    
    print("\n" + "="*60)
    print("🔄 模拟连续GATT通知流")
    print("="*60)
    
    # 模拟设备定期发送数据
    commands = [
        (0x0103, b'\x01\x00\x01\x00'),  # 寄存器数据1
        (0x0103, b'\x02\x00\x02\x00'),  # 寄存器数据2  
        (0x0171, b'V1.0.0'),           # 版本信息
        (0x0000, b'OK'),               # 状态响应
    ]
    
    for i in range(5):  # 发送5轮数据
        print(f"\n--- 第 {i+1} 轮数据传输 ---")
        
        for cmd, data in commands:
            packet = parser.create_packet(cmd, data)
            
            # 随机决定是否合并数据包 (模拟粘包)
            import random
            if random.random() > 0.5:  # 50%概率合并
                # 合并下一个包
                next_cmd, next_data = commands[(commands.index((cmd, data)) + 1) % len(commands)]
                next_packet = parser.create_packet(next_cmd, next_data)
                combined = packet + next_packet
                parser.process_gatt_data(combined)
            else:
                # 单独发送
                parser.process_gatt_data(packet)
            
            time.sleep(0.1)  # 模拟间隔
    
    print(f"\n📊 统计: 总共接收 {parser.total_received} 字节，解析出 {len(parser.received_packets)} 个数据包")


def interactive_bluetooth_test():
    """交互式蓝牙协议测试"""
    parser = BluetoothProtocolTestParser()
    
    print("\n" + "="*60)
    print("🎮 交互式蓝牙GATT协议测试")
    print("="*60)
    print("输入格式: <cmd_hex> [data_hex]")
    print("示例: 0103 01000100")
    print("输入 'stream' 模拟连续数据流")
    print("输入 'quit' 退出")
    
    while True:
        try:
            user_input = input("\n> ").strip()
            
            if user_input.lower() == 'quit':
                break
            elif user_input.lower() == 'stream':
                simulate_gatt_notification_stream()
                continue
            
            if not user_input:
                continue
            
            parts = user_input.split()
            cmd = int(parts[0], 16)
            data = bytes.fromhex(parts[1]) if len(parts) > 1 else b''
            
            # 创建数据包
            packet = parser.create_packet(cmd, data)
            print(f"创建GATT数据包: {packet.hex()}")
            
            # 解析数据包
            parsed = parser.process_gatt_data(packet)
            for p in parsed:
                parser.print_packet(p)
                
        except ValueError as e:
            print(f"输入错误: {e}")
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    # 运行蓝牙GATT粘包测试场景
    test_bluetooth_gatt_scenarios()
    
    # 运行连续数据流模拟
    simulate_gatt_notification_stream()
    
    # 运行交互式测试
    print("\n" + "="*60)
    interactive_bluetooth_test()
