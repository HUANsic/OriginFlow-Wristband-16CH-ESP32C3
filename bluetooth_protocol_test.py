#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OriginFlow 手环蓝牙GATT协议测试工具
扫描蓝牙广播、连接指定设备并通过GATT通知接收数据测试协议
"""

import asyncio
import struct
import time
from typing import Optional, List, Dict
from bleak import BleakScanner, BleakClient
from bleak.backends.device import BLEDevice
from bleak.backends.characteristic import BleakGATTCharacteristic

# 协议常量定义
FRAME_HEADER = 0xAA
PROTOCOL_VERSION = "V001"
FIXED_LEN = 17  # 固定包头长度
FIXED_WITHOUT_CRC_LEN = 15

# GATT服务UUID
SERVICE_UUID = "FB349B5F-8000-0080-0010-0000FF0001"
CHARACTERISTIC_UUID = "FB349B5F-8000-0080-0010-0000FF0002"


class BluetoothProtocolParser:
    """蓝牙协议解析器类"""
    
    def __init__(self):
        self.buffer = bytearray()
        self.received_packets = []
        
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
    
    def parse_packet(self, packet: bytes) -> Optional[Dict]:
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
    
    def process_data(self, new_data: bytes) -> List[Dict]:
        """处理新数据，处理粘包问题"""
        self.buffer.extend(new_data)
        packets = []
        
        print(f"接收新数据: {new_data.hex()}")
        print(f"当前缓冲区: {self.buffer.hex()}")
        
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


class BluetoothProtocolTester:
    """蓝牙协议测试器"""
    
    def __init__(self, target_device_name: str = None):
        self.target_device_name = target_device_name
        self.client: Optional[BleakClient] = None
        self.parser = BluetoothProtocolParser()
        self.device: Optional[BLEDevice] = None
        self.is_connected = False
        
    async def scan_devices(self, duration: float = 10.0) -> List[BLEDevice]:
        """扫描蓝牙设备"""
        print(f"扫描蓝牙设备 {duration} 秒...")
        devices = await BleakScanner.discover(timeout=duration)
        
        print(f"发现 {len(devices)} 个设备:")
        originflow_devices = []
        
        for i, device in enumerate(devices):
            device_info = f"  {i+1}. {device.name or 'Unknown'}"
            if device.address:
                device_info += f" ({device.address})"
            print(device_info)
            
            # 查找OriginFlow设备
            if device.name and ("OriginFlow" in device.name or 
                               (self.target_device_name and self.target_device_name in device.name)):
                originflow_devices.append(device)
        
        return originflow_devices
    
    async def connect_device(self, device: BLEDevice) -> bool:
        """连接到指定设备"""
        print(f"\n尝试连接到设备: {device.name} ({device.address})")
        
        try:
            self.client = BleakClient(device, timeout=10.0)
            await self.client.connect()
            
            if self.client.is_connected:
                self.device = device
                self.is_connected = True
                print(f"成功连接到设备: {device.name}")
                
                # 检查服务和特征
                await self.discover_services()
                return True
            else:
                print("连接失败")
                return False
                
        except Exception as e:
            print(f"连接错误: {e}")
            return False
    
    async def discover_services(self):
        """发现服务和特征"""
        if not self.client or not self.is_connected:
            print("设备未连接")
            return
        
        print("\n发现服务和特征:")
        
        # 检查目标服务
        service_found = False
        for service in self.client.services:
            print(f"  服务: {service.uuid}")
            
            if str(service.uuid).upper() == SERVICE_UUID.upper():
                print(f"    ✓ 找到目标服务: {service.uuid}")
                service_found = True
                
                for char in service.characteristics:
                    print(f"      特征: {char.uuid}")
                    print(f"        属性: {char.properties}")
                    
                    if str(char.uuid).upper() == CHARACTERISTIC_UUID.upper():
                        print(f"        ✓ 找到目标特征: {char.uuid}")
                        
                        # 启用通知
                        if "notify" in char.properties:
                            await self.enable_notifications(char)
        
        if not service_found:
            print(f"  ✗ 未找到目标服务: {SERVICE_UUID}")
    
    async def enable_notifications(self, characteristic: BleakGATTCharacteristic):
        """启用GATT通知"""
        try:
            print(f"\n启用特征通知: {characteristic.uuid}")
            
            def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
                print(f"\n收到GATT通知: {data.hex()}")
                # 处理协议数据
                packets = self.parser.process_data(data)
                for packet in packets:
                    self.parser.print_packet(packet)
            
            await self.client.start_notify(characteristic.uuid, notification_handler)
            print("✓ 通知已启用")
            
        except Exception as e:
            print(f"启用通知失败: {e}")
    
    async def send_command(self, cmd: int, data: bytes = None):
        """发送命令到设备"""
        if not self.client or not self.is_connected:
            print("设备未连接")
            return
        
        # 创建协议数据包
        packet = self.parser.create_packet(cmd, data)
        print(f"发送命令: 0x{cmd:04X} ({self.parser.get_cmd_name(cmd)})")
        print(f"数据包: {packet.hex()}")
        
        try:
            # 发送到GATT特征
            await self.client.write_gatt_char(CHARACTERISTIC_UUID, packet)
            print("✓ 命令已发送")
            
        except Exception as e:
            print(f"发送命令失败: {e}")
    
    async def disconnect(self):
        """断开连接"""
        if self.client and self.is_connected:
            await self.client.disconnect()
            self.is_connected = False
            print("设备已断开连接")
    
    async def interactive_mode(self):
        """交互式模式"""
        print("\n" + "="*60)
        print("OriginFlow 蓝牙协议测试工具 - 交互式模式")
        print("="*60)
        print("\n可用命令:")
        print("  scan                    - 扫描设备")
        print("  connect <index>         - 连接到设备")
        print("  send <cmd> [data]       - 发送命令")
        print("  disconnect              - 断开连接")
        print("  quit                    - 退出程序")
        print("  help                    - 显示帮助")
        print("\n常用命令:")
        print("  send 0x0102 0x01000100 - 读取寄存器")
        print("  send 0x00FF             - 重启设备")
        print("  send 0x0171             - 获取软件版本")
        
        while True:
            try:
                cmd_input = input("\n> ").strip()
                
                if cmd_input.lower() == 'quit':
                    break
                elif cmd_input.lower() == 'help':
                    print("可用命令: scan, connect, send, disconnect, quit, help")
                    continue
                elif cmd_input.lower() == 'scan':
                    devices = await self.scan_devices()
                    if devices:
                        print(f"\n找到 {len(devices)} 个OriginFlow设备:")
                        for i, device in enumerate(devices):
                            print(f"  {i+1}. {device.name} ({device.address})")
                    continue
                elif cmd_input.lower() == 'disconnect':
                    await self.disconnect()
                    continue
                elif cmd_input.startswith('connect '):
                    parts = cmd_input.split()
                    if len(parts) >= 2:
                        try:
                            index = int(parts[1]) - 1
                            devices = await self.scan_devices()
                            if 0 <= index < len(devices):
                                await self.connect_device(devices[index])
                            else:
                                print("无效的设备索引")
                        except ValueError:
                            print("无效的索引")
                    else:
                        print("用法: connect <index>")
                    continue
                elif cmd_input.startswith('send '):
                    parts = cmd_input.split()
                    if len(parts) >= 2:
                        try:
                            cmd = int(parts[1], 16)
                            data = bytes.fromhex(parts[2]) if len(parts) > 2 else None
                            await self.send_command(cmd, data)
                        except ValueError:
                            print("无效的16进制数值")
                    else:
                        print("用法: send <cmd> [data]")
                    continue
                else:
                    print("未知命令，输入 'help' 查看帮助")
                    
            except KeyboardInterrupt:
                print("\n程序被中断")
                break
            except Exception as e:
                print(f"错误: {e}")


async def main():
    """主函数"""
    print("OriginFlow 手环蓝牙GATT协议测试工具")
    print("="*60)
    
    # 创建测试器
    tester = BluetoothProtocolTester()
    
    # 扫描设备
    print("\n扫描蓝牙设备...")
    devices = await tester.scan_devices(15.0)
    
    if not devices:
        print("未找到OriginFlow设备，请检查设备是否在广播状态")
        print("继续运行交互模式，你可以手动扫描...")
    else:
        print(f"\n找到 {len(devices)} 个OriginFlow设备:")
        for i, device in enumerate(devices):
            print(f"  {i+1}. {device.name} ({device.address})")
        
        # 自动连接第一个设备
        print(f"\n自动连接到第一个设备: {devices[0].name}")
        if await tester.connect_device(devices[0]):
            print("✓ 设备连接成功，可以开始发送命令")
        else:
            print("✗ 设备连接失败")
    
    # 进入交互式模式
    await tester.interactive_mode()
    
    # 清理连接
    await tester.disconnect()


if __name__ == "__main__":
    # 检查 bleak 是否安装
    try:
        import bleak
    except ImportError:
        print("错误: 需要安装 bleak 库")
        print("请运行: pip install bleak")
        exit(1)
    
    # 运行主程序
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n程序被用户中断")
