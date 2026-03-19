import asyncio
import struct
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import threading
import sys
import queue
from typing import Optional, List, Dict
import serial
import argparse

# 设置Windows事件循环策略（Windows专用）
if sys.platform == "win32":
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

from bleak import BleakScanner, BleakClient
from bleak.backends.device import BLEDevice
from bleak.backends.characteristic import BleakGATTCharacteristic

# 设置matplotlib后端和中文字体
import matplotlib
matplotlib.use('TkAgg')
# 优先使用系统中文字体，避免字体缺失警告
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 协议常量定义
FRAME_HEADER = 0xAA
PROTOCOL_VERSION = "V001"
FIXED_LEN = 17  # 固定包头长度
FIXED_WITHOUT_CRC_LEN = 15

# GATT服务UUID
SERVICE_UUID = "0100ff00-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "0200ff00-0000-1000-8000-00805f9b34fb"


class SerialWorker:
    """串口通信工作线程"""
    
    def __init__(self, data_queue, port=None, baudrate=115200, parser=None):
        self.data_queue = data_queue
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.parser = parser or BluetoothProtocolParser()  # 使用外部解析器或创建新的
        self.attitude_estimator = IMUAttitudeEstimator()
        self.should_run = False
        
    def connect(self):
        """连接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"✓ 成功连接串口: {self.port} @ {self.baudrate} bps")
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("✓ 串口已断开连接")
    
    def read_data(self):
        """读取串口数据"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                # 检查是否有数据可读
                if self.serial_conn.in_waiting > 0:
                    # 读取所有可用数据
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    return data
        except Exception as e:
            print(f"串口读取错误: {e}")
            self.disconnect()
        return None
    
    def process_serial_data(self, data):
        """处理串口数据"""
        if data:
            # 处理协议数据
            packets = self.parser.process_data(data)
            for packet in packets:
                self.parser.print_packet(packet, self.attitude_estimator, self.data_queue)
    
    def run_serial(self):
        """运行串口任务"""
        print(f"开始监听串口数据...")
        
        try:
            while self.should_run:
                # 读取数据
                data = self.read_data()
                if data:
                    self.process_serial_data(data)
                
                # 短暂休眠避免CPU占用过高
                time.sleep(0.01)
                
        except Exception as e:
            print(f"串口任务错误: {e}")
        finally:
            self.disconnect()
    
    def start(self):
        """启动串口线程"""
        if not self.connect():
            return False
        
        self.should_run = True
        self.thread = threading.Thread(target=self.run_serial, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """停止串口线程"""
        self.should_run = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.disconnect()


class MagnetometerCalibrator:
    """磁力计椭圆拟合校准器"""
    
    def __init__(self):
        # 存储磁力计数据点
        self.mag_points = []
        self.max_points = 1000  # 最大存储点数
        
        # 椭圆参数
        self.center = np.array([0.0, 0.0])  # 椭圆中心
        self.a = 1.0  # 长轴半径
        self.b = 1.0  # 短轴半径
        self.angle = 0.0  # 椭圆旋转角度
        self.is_calibrated = False
        
        # 校准参数
        self.calibration_matrix = np.eye(3)  # 3x3校准矩阵
        self.bias = np.zeros(3)  # 偏移向量
        
        # 角度计算相关
        self.declination = 0.0  # 磁偏角（度）
        self.last_heading = 0.0  # 上次计算的航向角
        
    def set_magnetic_declination(self, declination_degrees):
        """设置磁偏角（用于补偿真北方向）"""
        self.declination = declination_degrees
        
    def calculate_heading(self, mag_x, mag_y, mag_z):
        """计算磁航向角（基于校准后的磁力计数据）"""
        # 校准磁力计数据
        if self.is_calibrated:
            mag_calibrated = self.calibrate_point(mag_x, mag_y, mag_z)
        else:
            mag_calibrated = [mag_x, mag_y, mag_z]
        
        # 使用校准后的X和Y计算航向角
        mx, my, mz = mag_calibrated
        
        # 计算航向角（弧度）
        heading_rad = np.arctan2(my, mx)
        
        # 转换为角度（0-360度）
        heading_deg = np.degrees(heading_rad)
        if heading_deg < 0:
            heading_deg += 360
        
        # 应用磁偏角补偿
        heading_deg += self.declination
        if heading_deg >= 360:
            heading_deg -= 360
        elif heading_deg < 0:
            heading_deg += 360
        
        # 平滑处理（可选）
        # 可以在这里添加滤波算法来减少噪声
        
        return heading_deg
    
    def calculate_tilt_compensated_heading(self, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z):
        """计算倾斜补偿的航向角（需要加速度计数据）"""
        # 校准磁力计数据
        if self.is_calibrated:
            mag_calibrated = self.calibrate_point(mag_x, mag_y, mag_z)
        else:
            mag_calibrated = [mag_x, mag_y, mag_z]
        
        mx, my, mz = mag_calibrated
        
        # 计算俯仰角和横滚角（来自加速度计）
        # 归一化加速度向量
        accel_norm = np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if accel_norm > 0:
            ax, ay, az = accel_x/accel_norm, accel_y/accel_norm, accel_z/accel_norm
        else:
            ax, ay, az = 0, 0, 1
        
        # 计算俯仰角和横滚角
        pitch = np.arcsin(-ax)  # 俯仰角
        roll = np.arctan2(ay, az)  # 横滚角
        
        # 倾斜补偿：将磁力计数据旋转到水平面
        # 绕Y轴旋转（俯仰补偿）
        mx_pitch = mx * np.cos(pitch) + mz * np.sin(pitch)
        my_pitch = my
        mz_pitch = -mx * np.sin(pitch) + mz * np.cos(pitch)
        
        # 绕X轴旋转（横滚补偿）
        mx_comp = mx_pitch
        my_comp = my_pitch * np.cos(roll) - mz_pitch * np.sin(roll)
        
        # 计算补偿后的航向角
        heading_rad = np.arctan2(my_comp, mx_comp)
        heading_deg = np.degrees(heading_rad)
        
        if heading_deg < 0:
            heading_deg += 360
        
        # 应用磁偏角补偿
        heading_deg += self.declination
        if heading_deg >= 360:
            heading_deg -= 360
        elif heading_deg < 0:
            heading_deg += 360
        
        return heading_deg
        
    def add_point(self, mag_x, mag_y, mag_z):
        """添加磁力计数据点"""
        # 只使用X和Y进行2D椭圆拟合
        self.mag_points.append([mag_x, mag_y])
        
        # 限制存储点数
        if len(self.mag_points) > self.max_points:
            self.mag_points.pop(0)
    
    def fit_ellipse(self):
        """使用最小二乘法拟合椭圆"""
        if len(self.mag_points) < 50:
            return False
        
        points = np.array(self.mag_points)
        x = points[:, 0]
        y = points[:, 1]
        
        # 构建设计矩阵 D = [x² xy y² x y 1]
        D = np.column_stack([x**2, x*y, y**2, x, y, np.ones_like(x)])
        
        # 解广义特征值问题 D^T D a = λ C a
        # 其中 C是约束矩阵
        C = np.array([
            [0, 0, 2, 0, 0, 0],
            [0, -1, 0, 0, 0, 0],
            [2, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]
        ])
        
        # 计算广义特征值和特征向量
        try:
            # 使用SVD求解约束最小二乘问题
            DTD = D.T @ D
            eigenvalues, eigenvectors = np.linalg.eig(np.linalg.solve(DTD, C))
            
            # 找到对应正特征值的特征向量
            idx = np.where(eigenvalues > 0)[0]
            if len(idx) == 0:
                return False
            
            # 选择最优解
            a = eigenvectors[:, idx[0]]
            
            # 提取椭圆参数
            A, B, C, D, E, F = a
            
            # 计算椭圆中心
            denominator = B**2 - 4*A*C
            if abs(denominator) < 1e-10:
                return False
            
            self.center[0] = (2*C*D - B*E) / denominator
            self.center[1] = (2*A*E - B*D) / denominator
            
            # 计算椭圆参数
            # 平移坐标系到椭圆中心
            x_centered = x - self.center[0]
            y_centered = y - self.center[1]
            
            # 计算旋转角度
            if abs(B) < 1e-10:
                self.angle = 0 if A < C else np.pi/2
            else:
                self.angle = 0.5 * np.arctan(B / (A - C))
            
            # 旋转坐标系
            cos_angle = np.cos(self.angle)
            sin_angle = np.sin(self.angle)
            x_rot = x_centered * cos_angle + y_centered * sin_angle
            y_rot = -x_centered * sin_angle + y_centered * cos_angle
            
            # 计算长轴和短轴
            self.a = np.sqrt(np.max(x_rot**2 + y_rot**2))
            self.b = np.sqrt(np.min(x_rot**2 + y_rot**2))
            
            # 计算校准参数
            self.calculate_calibration_parameters()
            
            self.is_calibrated = True
            return True
            
        except Exception as e:
            print(f"椭圆拟合失败: {e}")
            return False
    
    def calculate_calibration_parameters(self):
        """计算校准参数"""
        if not self.is_calibrated:
            return
        
        # 创建旋转矩阵
        cos_angle = np.cos(self.angle)
        sin_angle = np.sin(self.angle)
        rotation_matrix = np.array([
            [cos_angle, -sin_angle, 0],
            [sin_angle, cos_angle, 0],
            [0, 0, 1]
        ])
        
        # 创建缩放矩阵
        scale_matrix = np.array([
            [1.0/self.a, 0, 0],
            [0, 1.0/self.b, 0],
            [0, 0, 1.0]
        ])
        
        # 计算校准矩阵
        self.calibration_matrix = rotation_matrix.T @ scale_matrix @ rotation_matrix
        
        # 计算偏移（椭圆中心）
        self.bias = np.array([self.center[0], self.center[1], 0])
    
    def calibrate_point(self, mag_x, mag_y, mag_z):
        """校准磁力计数据点"""
        if not self.is_calibrated:
            return mag_x, mag_y, mag_z
        
        # 转换为numpy数组
        mag = np.array([mag_x, mag_y, mag_z])
        
        # 应用校准
        mag_calibrated = self.calibration_matrix @ (mag - self.bias)
        
        return mag_calibrated[0], mag_calibrated[1], mag_calibrated[2]
    
    def get_ellipse_points(self, num_points=100):
        """获取椭圆轮廓点用于绘制"""
        if not self.is_calibrated:
            return [], []
        
        # 生成椭圆点
        t = np.linspace(0, 2*np.pi, num_points)
        ellipse_x = self.a * np.cos(t)
        ellipse_y = self.b * np.sin(t)
        
        # 旋转椭圆
        cos_angle = np.cos(self.angle)
        sin_angle = np.sin(self.angle)
        
        rotated_x = ellipse_x * cos_angle - ellipse_y * sin_angle + self.center[0]
        rotated_y = ellipse_x * sin_angle + ellipse_y * cos_angle + self.center[1]
        
        return rotated_x, rotated_y
    
    def reset(self):
        """重置校准器"""
        self.mag_points.clear()
        self.is_calibrated = False
        self.center = np.array([0.0, 0.0])
        self.a = 1.0
        self.b = 1.0
        self.angle = 0.0
        self.calibration_matrix = np.eye(3)
        self.bias = np.zeros(3)


class IMUAttitudeEstimator:
    """6轴IMU姿态解算器"""
    
    def __init__(self):
        # 姿态四元数 [w, x, y, z]
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        
        # 互补滤波器参数
        self.alpha = 0.95  # 互补滤波器系数
        
        # 陀螺仪偏置
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        
        # 时间戳
        self.last_timestamp = None
        
        # 姿态角度 (roll, pitch, yaw) - 使用度数
        self.angles = np.array([0.0, 0.0, 0.0])
        
        # 陀螺仪标定参数
        self.gyro_scale = 0.001  # 度/秒转换为弧度/秒
        
    def normalize_quaternion(self, q):
        """四元数归一化"""
        norm = np.linalg.norm(q)
        if norm > 0:
            return q / norm
        return q
    
    def quaternion_to_euler(self, q):
        """四元数转欧拉角"""
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])
    
    def update_imu(self, gyro, accel, timestamp=None):
        """更新IMU数据并计算姿态"""
        if timestamp is None:
            timestamp = time.time()
        
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            return self.angles
        
        dt = timestamp - self.last_timestamp
        self.last_timestamp = timestamp
        
        # 陀螺仪数据 - 假设输入是度/秒，转换为弧度/秒
        gx, gy, gz = gyro * self.gyro_scale
        
        # 加速度计数据 - 假设输入是mg，转换为m/s²
        ax, ay, az = accel / 1000.0 * 9.81
        
        # 从加速度计计算姿态（仅用于roll和pitch）
        # 归一化加速度向量
        accel_norm = np.linalg.norm([ax, ay, az])
        if accel_norm > 0:
            ax, ay, az = ax/accel_norm, ay/accel_norm, az/accel_norm
        
        # 从加速度计计算角度
        accel_roll = np.arctan2(ay, np.sqrt(ax**2 + az**2))
        accel_pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        
        # 从陀螺仪积分计算姿态变化
        gyro_roll = gx * dt
        gyro_pitch = gy * dt
        gyro_yaw = gz * dt
        
        # 互补滤波器融合陀螺仪和加速度计数据
        self.angles[0] = self.alpha * (self.angles[0] + np.degrees(gyro_roll)) + (1 - self.alpha) * np.degrees(accel_roll)
        self.angles[1] = self.alpha * (self.angles[1] + np.degrees(gyro_pitch)) + (1 - self.alpha) * np.degrees(accel_pitch)
        self.angles[2] += np.degrees(gyro_yaw)  # yaw角只能从陀螺仪获得
        
        # 角度归一化到[-180, 180]范围
        self.angles[0] = self.normalize_angle(self.angles[0])
        self.angles[1] = self.normalize_angle(self.angles[1])
        self.angles[2] = self.normalize_angle(self.angles[2])
        
        return self.angles
    
    def normalize_angle(self, angle):
        """将角度归一化到[-180, 180]范围"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle


class AttitudeVisualizer:
    """姿态可视化界面 - 主线程运行"""
    
    def __init__(self, data_queue):
        # 数据队列
        self.data_queue = data_queue
        
        # 创建磁力计校准器
        self.mag_calibrator = MagnetometerCalibrator()
        
        # 创建图形界面 - 调整为3x4布局
        self.fig = plt.figure(figsize=(20, 12))
        self.fig.suptitle('9轴IMU姿态实时显示 - 带磁力计椭圆拟合校准和角度计算', fontsize=16)
        
        # 3D姿态显示
        self.ax_3d = self.fig.add_subplot(341, projection='3d')
        self.ax_3d.set_title('3D姿态')
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_xlim([-2, 2])
        self.ax_3d.set_ylim([-2, 2])
        self.ax_3d.set_zlim([-2, 2])
        
        # 欧拉角时间序列
        self.ax_euler = self.fig.add_subplot(342)
        self.ax_euler.set_title('欧拉角')
        self.ax_euler.set_xlabel('时间 (s)')
        self.ax_euler.set_ylabel('角度 (°)')
        self.ax_euler.set_ylim([-180, 180])
        self.ax_euler.grid(True)
        
        # 加速度计显示
        self.ax_accel = self.fig.add_subplot(343)
        self.ax_accel.set_title('加速度计')
        self.ax_accel.set_xlabel('轴')
        self.ax_accel.set_ylabel('加速度 (m/s²)')
        self.ax_accel.set_ylim([-20, 20])
        self.ax_accel.grid(True)
        
        # 陀螺仪显示
        self.ax_gyro = self.fig.add_subplot(344)
        self.ax_gyro.set_title('陀螺仪')
        self.ax_gyro.set_xlabel('轴')
        self.ax_gyro.set_ylabel('角速度 (°/s)')
        self.ax_gyro.set_ylim([-2000, 2000])
        self.ax_gyro.grid(True)
        
        # 磁力计显示
        self.ax_mag = self.fig.add_subplot(345)
        self.ax_mag.set_title('磁力计')
        self.ax_mag.set_xlabel('轴')
        self.ax_mag.set_ylabel('磁场强度 (μT)')
        self.ax_mag.set_ylim([-100, 100])
        self.ax_mag.grid(True)
        
        # 磁力计椭圆拟合显示
        self.ax_mag_ellipse = self.fig.add_subplot(346)
        self.ax_mag_ellipse.set_title('磁力计椭圆拟合 (X-Y平面)')
        self.ax_mag_ellipse.set_xlabel('磁场 X (μT)')
        self.ax_mag_ellipse.set_ylabel('磁场 Y (μT)')
        self.ax_mag_ellipse.set_aspect('equal')
        self.ax_mag_ellipse.grid(True)
        
        # 校准后磁力计显示
        self.ax_mag_calibrated = self.fig.add_subplot(347)
        self.ax_mag_calibrated.set_title('校准后磁力计')
        self.ax_mag_calibrated.set_xlabel('轴')
        self.ax_mag_calibrated.set_ylabel('磁场强度 (归一化)')
        self.ax_mag_calibrated.set_ylim([-1.5, 1.5])
        self.ax_mag_calibrated.grid(True)
        
        # 航向角时间序列
        self.ax_heading = self.fig.add_subplot(348)
        self.ax_heading.set_title('磁航向角')
        self.ax_heading.set_xlabel('时间 (s)')
        self.ax_heading.set_ylabel('航向角 (°)')
        self.ax_heading.set_ylim([0, 360])
        self.ax_heading.grid(True)
        
        # 传感器数据对比图
        self.ax_compare = self.fig.add_subplot(349)
        self.ax_compare.set_title('传感器数据对比')
        self.ax_compare.set_xlabel('传感器轴')
        self.ax_compare.set_ylabel('归一化值')
        self.ax_compare.set_ylim([-1.2, 1.2])
        self.ax_compare.grid(True)
        
        # 校准状态显示
        self.ax_status = self.fig.add_subplot(3,4,10)
        self.ax_status.set_title('校准状态')
        self.ax_status.axis('off')
        
        # 罗盘显示
        self.ax_compass = self.fig.add_subplot(3,4,11, projection='polar')
        self.ax_compass.set_title('磁罗盘')
        
        # 角度对比图
        self.ax_angle_compare = self.fig.add_subplot(3,4,12)
        self.ax_angle_compare.set_title('角度对比')
        self.ax_angle_compare.set_ylabel('角度 (°)')
        self.ax_angle_compare.set_ylim([0, 360])
        self.ax_angle_compare.grid(True)
        
        # FPS历史图表 - 调整布局，使用现有位置
        # 将FPS显示集成到校准状态面板中，避免超出3x4限制
        
        # 数据缓存
        self.max_points = 200
        self.time_data = deque(maxlen=self.max_points)
        self.roll_data = deque(maxlen=self.max_points)
        self.pitch_data = deque(maxlen=self.max_points)
        self.yaw_data = deque(maxlen=self.max_points)
        
        # 当前数据
        self.current_angles = [0, 0, 0]
        self.current_accel = [0, 0, 0]
        self.current_gyro = [0, 0, 0]
        self.current_mag = [0, 0, 0]
        self.current_mag_calibrated = [0, 0, 0]
        
        # 磁力计角度数据
        self.current_mag_heading = 0.0  # 磁航向角
        self.current_mag_heading_tilt_compensated = 0.0  # 倾斜补偿航向角
        
        # 磁力计历史数据（用于椭圆拟合显示）
        self.mag_history_x = deque(maxlen=500)
        self.mag_history_y = deque(maxlen=500)
        
        # 航向角历史数据
        self.heading_history = deque(maxlen=200)
        self.heading_tilt_compensated_history = deque(maxlen=200)
        
        # 启动时间
        self.start_time = time.time()
        
        # 运行状态
        self.is_running = True
        
        # 校准控制
        self.calibration_counter = 0
        self.auto_calibrate = True
        
        # 帧率统计（简化版本，只显示数值）
        self.frame_count = 0
        self.fps_start_time = time.time()
        self.current_fps = 0.0
        self.fps_update_interval = 1.0  # 每秒更新一次FPS
        self.last_fps_update = time.time()
        
        plt.tight_layout()
        
    def update_data(self):
        """从队列更新数据"""
        data_received = False
        
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                if len(data) == 4:  # (angles, accel, gyro, mag)
                    angles, accel, gyro, mag = data
                else:  # 兼容旧格式 (angles, accel, gyro)
                    angles, accel, gyro = data
                    mag = [0, 0, 0]  # 默认磁力计数据
                
                data_received = True
                current_time = time.time() - self.start_time
                
                self.current_angles = angles
                self.current_accel = accel
                self.current_gyro = gyro
                self.current_mag = mag
                
                # 更新帧率统计
                self.frame_count += 1
                
                # 磁力计校准处理
                if not np.array_equal(mag, [0, 0, 0]):  # 有磁力计数据
                    # 添加数据点到校准器
                    self.mag_calibrator.add_point(mag[0], mag[1], mag[2])
                    
                    # 更新磁力计历史数据
                    self.mag_history_x.append(mag[0])
                    self.mag_history_y.append(mag[1])
                    
                    # 校准磁力计数据
                    mag_calibrated = self.mag_calibrator.calibrate_point(mag[0], mag[1], mag[2])
                    self.current_mag_calibrated = list(mag_calibrated)
                    
                    # 计算磁航向角
                    self.current_mag_heading = self.mag_calibrator.calculate_heading(mag[0], mag[1], mag[2])
                    
                    # 计算倾斜补偿航向角（如果有加速度计数据）
                    if not np.array_equal(accel, [0, 0, 0]):
                        self.current_mag_heading_tilt_compensated = self.mag_calibrator.calculate_tilt_compensated_heading(
                            mag[0], mag[1], mag[2], accel[0], accel[1], accel[2])
                    else:
                        self.current_mag_heading_tilt_compensated = self.current_mag_heading
                    
                    # 更新航向角历史数据
                    self.heading_history.append(self.current_mag_heading)
                    self.heading_tilt_compensated_history.append(self.current_mag_heading_tilt_compensated)
                    
                    # 自动校准：每100个数据点尝试拟合一次
                    self.calibration_counter += 1
                    if self.auto_calibrate and self.calibration_counter % 100 == 0:
                        if self.mag_calibrator.fit_ellipse():
                            print(f"✓ 磁力计椭圆拟合成功！椭圆参数: 中心({self.mag_calibrator.center[0]:.2f}, {self.mag_calibrator.center[1]:.2f}), 长轴{self.mag_calibrator.a:.2f}, 短轴{self.mag_calibrator.b:.2f}, 角度{np.degrees(self.mag_calibrator.angle):.2f}°")
                        else:
                            print(f"磁力计椭圆拟合失败，当前数据点数: {len(self.mag_calibrator.mag_points)}")
                else:
                    self.current_mag_calibrated = [0, 0, 0]
                    self.current_mag_heading = 0.0
                    self.current_mag_heading_tilt_compensated = 0.0
                
                # 更新时间序列数据
                self.time_data.append(current_time)
                self.roll_data.append(angles[0])
                self.pitch_data.append(angles[1])
                self.yaw_data.append(angles[2])
            except queue.Empty:
                break
        
        # 更新FPS统计
        current_time = time.time()
        if current_time - self.last_fps_update >= self.fps_update_interval:
            elapsed_time = current_time - self.fps_start_time
            if elapsed_time > 0:
                self.current_fps = self.frame_count / elapsed_time
                
                # 重置统计
                self.frame_count = 0
                self.fps_start_time = current_time
                self.last_fps_update = current_time
    
    def draw_3d_attitude(self):
        """绘制3D姿态"""
        self.ax_3d.clear()
        self.ax_3d.set_title('3D姿态')
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_xlim([-2, 2])
        self.ax_3d.set_ylim([-2, 2])
        self.ax_3d.set_zlim([-2, 2])
        
        # 创建旋转矩阵
        roll, pitch, yaw = np.radians(self.current_angles)
        
        # Rotation matrix around Z (yaw)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Rotation matrix around Y (pitch)
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Rotation matrix around X (roll)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Combined rotation matrix
        R = Rz @ Ry @ Rx
        
        # 绘制坐标系
        origin = np.array([0, 0, 0])
        
        # X-axis (red)
        x_axis = R @ np.array([1, 0, 0])
        self.ax_3d.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], 
                         color='r', arrow_length_ratio=0.1, linewidth=2)
        
        # Y-axis (green)
        y_axis = R @ np.array([0, 1, 0])
        self.ax_3d.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], 
                         color='g', arrow_length_ratio=0.1, linewidth=2)
        
        # Z-axis (blue)
        z_axis = R @ np.array([0, 0, 1])
        self.ax_3d.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], 
                         color='b', arrow_length_ratio=0.1, linewidth=2)
        
        # 绘制立方体框架
        self.draw_cube_frame(R)
    
    def draw_cube_frame(self, R):
        """绘制立方体框架"""
        # 立方体顶点
        vertices = np.array([
            [-0.5, -0.5, -0.5],
            [0.5, -0.5, -0.5],
            [0.5, 0.5, -0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, -0.5, 0.5],
            [0.5, -0.5, 0.5],
            [0.5, 0.5, 0.5],
            [-0.5, 0.5, 0.5]
        ])
        
        # 旋转顶点
        rotated_vertices = (R @ vertices.T).T
        
        # 绘制边
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # 底面
            [4, 5], [5, 6], [6, 7], [7, 4],  # 顶面
            [0, 4], [1, 5], [2, 6], [3, 7]   # 垂直边
        ]
        
        for edge in edges:
            points = rotated_vertices[edge]
            self.ax_3d.plot3D(*points.T, 'k-', linewidth=1)
    
    def update_plots(self, frame):
        """更新所有图表 - matplotlib动画回调"""
        if not self.is_running:
            return []
        
        # 更新数据
        self.update_data()
        
        # 清除并重新绘制3D姿态
        self.draw_3d_attitude()
        
        # 更新欧拉角图
        self.ax_euler.clear()
        self.ax_euler.set_title('欧拉角')
        self.ax_euler.set_xlabel('时间 (s)')
        self.ax_euler.set_ylabel('角度 (°)')
        self.ax_euler.set_ylim([-180, 180])
        self.ax_euler.grid(True)
        
        if len(self.time_data) > 0:
            self.ax_euler.plot(self.time_data, self.roll_data, 'r-', label='Roll', linewidth=2)
            self.ax_euler.plot(self.time_data, self.pitch_data, 'g-', label='Pitch', linewidth=2)
            self.ax_euler.plot(self.time_data, self.yaw_data, 'b-', label='Yaw', linewidth=2)
            self.ax_euler.legend()
        
        # 更新加速度计图
        self.ax_accel.clear()
        self.ax_accel.set_title('加速度计')
        self.ax_accel.set_xlabel('轴')
        self.ax_accel.set_ylabel('加速度 (m/s²)')
        self.ax_accel.set_ylim([-20, 20])
        self.ax_accel.grid(True)
        
        axes = ['X', 'Y', 'Z']
        colors = ['r', 'g', 'b']
        for i, (axis, color) in enumerate(zip(axes, colors)):
            self.ax_accel.bar(axis, self.current_accel[i], color=color, alpha=0.7)
        
        # 更新陀螺仪图
        self.ax_gyro.clear()
        self.ax_gyro.set_title('陀螺仪')
        self.ax_gyro.set_xlabel('轴')
        self.ax_gyro.set_ylabel('角速度 (°/s)')
        self.ax_gyro.set_ylim([-2000, 2000])
        self.ax_gyro.grid(True)
        
        for i, (axis, color) in enumerate(zip(axes, colors)):
            self.ax_gyro.bar(axis, self.current_gyro[i], color=color, alpha=0.7)
        
        # 更新磁力计图
        self.ax_mag.clear()
        self.ax_mag.set_title('磁力计')
        self.ax_mag.set_xlabel('轴')
        self.ax_mag.set_ylabel('磁场强度 (μT)')
        self.ax_mag.set_ylim([-100, 100])
        self.ax_mag.grid(True)
        
        for i, (axis, color) in enumerate(zip(axes, colors)):
            self.ax_mag.bar(axis, self.current_mag[i], color=color, alpha=0.7)
        
        # 更新磁力计椭圆拟合图
        self.ax_mag_ellipse.clear()
        self.ax_mag_ellipse.set_title('磁力计椭圆拟合 (X-Y平面)')
        self.ax_mag_ellipse.set_xlabel('磁场 X (μT)')
        self.ax_mag_ellipse.set_ylabel('磁场 Y (μT)')
        self.ax_mag_ellipse.grid(True)
        
        if len(self.mag_history_x) > 0:
            # 绘制历史数据点
            self.ax_mag_ellipse.scatter(self.mag_history_x, self.mag_history_y, 
                                      c='blue', s=1, alpha=0.6, label='原始数据')
        
        # 如果已校准，绘制拟合椭圆
        if self.mag_calibrator.is_calibrated:
            ellipse_x, ellipse_y = self.mag_calibrator.get_ellipse_points()
            self.ax_mag_ellipse.plot(ellipse_x, ellipse_y, 'r-', linewidth=2, label='拟合椭圆')
            
            # 绘制椭圆中心
            self.ax_mag_ellipse.scatter([self.mag_calibrator.center[0]], [self.mag_calibrator.center[1]], 
                                       c='red', s=50, marker='x', linewidth=2, label='椭圆中心')
            
            # 绘制当前点
            if len(self.current_mag) >= 2:
                self.ax_mag_ellipse.scatter([self.current_mag[0]], [self.current_mag[1]], 
                                          c='green', s=50, marker='o', label='当前点')
        
        self.ax_mag_ellipse.legend()
        
        # 更新校准后磁力计图
        self.ax_mag_calibrated.clear()
        self.ax_mag_calibrated.set_title('校准后磁力计')
        self.ax_mag_calibrated.set_xlabel('轴')
        self.ax_mag_calibrated.set_ylabel('磁场强度 (归一化)')
        self.ax_mag_calibrated.set_ylim([-1.5, 1.5])
        self.ax_mag_calibrated.grid(True)
        
        for i, (axis, color) in enumerate(zip(axes, colors)):
            self.ax_mag_calibrated.bar(axis, self.current_mag_calibrated[i], color=color, alpha=0.7)
        
        # 更新校准状态显示
        self.ax_status.clear()
        self.ax_status.set_title('校准状态')
        self.ax_status.axis('off')
        
        # 从协议解析器获取FPS
        imu_fps = self.parser.get_imu_fps() if hasattr(self, 'parser') else 0.0
        
        status_text = f"数据点数: {len(self.mag_calibrator.mag_points)}\n"
        status_text += f"校准状态: {'已校准' if self.mag_calibrator.is_calibrated else '未校准'}\n"
        status_text += f"磁航向角: {self.current_mag_heading:.1f}°\n"
        status_text += f"补偿航向角: {self.current_mag_heading_tilt_compensated:.1f}°\n"
        status_text += f"IMU接收帧率: {imu_fps:.1f} FPS\n"
        
        if self.mag_calibrator.is_calibrated:
            status_text += f"椭圆中心: ({self.mag_calibrator.center[0]:.2f}, {self.mag_calibrator.center[1]:.2f})\n"
            status_text += f"长轴: {self.mag_calibrator.a:.2f} μT\n"
            status_text += f"短轴: {self.mag_calibrator.b:.2f} μT\n"
            status_text += f"旋转角: {np.degrees(self.mag_calibrator.angle):.2f}°\n"
            status_text += f"椭圆率: {self.mag_calibrator.b/self.mag_calibrator.a:.3f}"
        else:
            status_text += "请旋转设备进行校准\n"
            status_text += f"需要至少50个数据点"
        
        self.ax_status.text(0.1, 0.5, status_text, fontsize=10, 
                           verticalalignment='center')
        
        # 更新航向角时间序列图
        self.ax_heading.clear()
        self.ax_heading.set_title('磁航向角')
        self.ax_heading.set_xlabel('时间 (s)')
        self.ax_heading.set_ylabel('航向角 (°)')
        self.ax_heading.set_ylim([0, 360])
        self.ax_heading.grid(True)
        
        if len(self.time_data) > 0 and len(self.heading_history) > 0:
            # 确保时间数据长度匹配
            heading_time = list(self.time_data)[-len(self.heading_history):]
            self.ax_heading.plot(heading_time, list(self.heading_history), 'b-', 
                               label='磁航向角', linewidth=2)
            self.ax_heading.plot(heading_time, list(self.heading_tilt_compensated_history), 'r--', 
                               label='倾斜补偿航向角', linewidth=2)
            self.ax_heading.legend()
        
        # 更新罗盘显示
        self.ax_compass.clear()
        self.ax_compass.set_title('磁罗盘')
        
        # 转换航向角为弧度（0度在北方，顺时针增加）
        heading_rad = np.radians(self.current_mag_heading)
        heading_tilt_rad = np.radians(self.current_mag_heading_tilt_compensated)
        
        # 绘制罗盘指针
        self.ax_compass.arrow(0, 0, heading_rad, 0.8, head_width=0.1, 
                             head_length=0.1, fc='blue', ec='blue', 
                             label='磁航向', linewidth=2)
        self.ax_compass.arrow(0, 0, heading_tilt_rad, 0.6, head_width=0.08, 
                             head_length=0.08, fc='red', ec='red', 
                             label='补偿航向', linewidth=1.5, alpha=0.7)
        
        # 设置罗盘方向
        self.ax_compass.set_theta_zero_location('N')
        self.ax_compass.set_theta_direction(-1)
        self.ax_compass.set_ylim(0, 1)
        
        # 添加方向标签（中文）
        directions = ['北', '东北', '东', '东南', '南', '西南', '西', '西北']
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
        self.ax_compass.set_thetagrids(angles, directions)
        
        # 更新角度对比图
        self.ax_angle_compare.clear()
        self.ax_angle_compare.set_title('角度对比')
        self.ax_angle_compare.set_ylabel('角度 (°)')
        self.ax_angle_compare.set_ylim([0, 360])
        self.ax_angle_compare.grid(True)
        
        angles_labels = ['陀螺仪Yaw', '磁航向角', '补偿航向角']
        angles_values = [
            self.current_angles[2] % 360,  # 陀螺仪Yaw角
            self.current_mag_heading,       # 磁航向角
            self.current_mag_heading_tilt_compensated  # 倾斜补偿航向角
        ]
        colors = ['green', 'blue', 'red']
        
        bars = self.ax_angle_compare.bar(angles_labels, angles_values, color=colors, alpha=0.7)
        
        # 在柱状图上显示数值
        for bar, value in zip(bars, angles_values):
            height = bar.get_height()
            self.ax_angle_compare.text(bar.get_x() + bar.get_width()/2., height + 5,
                                     f'{value:.1f}°', ha='center', va='bottom', fontsize=10)
        
        # 更新传感器数据对比图
        self.ax_compare.clear()
        self.ax_compare.set_title('传感器数据对比')
        self.ax_compare.set_xlabel('传感器类型')
        self.ax_compare.set_ylabel('归一化值')
        self.ax_compare.set_ylim([-1.2, 1.2])
        self.ax_compare.grid(True)
        
        # 归一化传感器数据
        accel_norm = np.array(self.current_accel) / 20.0  # 归一化到[-1,1]
        gyro_norm = np.array(self.current_gyro) / 2000.0  # 归一化到[-1,1]
        mag_norm = np.array(self.current_mag) / 100.0    # 归一化到[-1,1]
        
        # 创建分组柱状图
        x = np.arange(3)
        width = 0.25
        
        self.ax_compare.bar(x - width, accel_norm, width, label='加速度计', color='r', alpha=0.7)
        self.ax_compare.bar(x, gyro_norm, width, label='陀螺仪', color='g', alpha=0.7)
        self.ax_compare.bar(x + width, mag_norm, width, label='磁力计', color='b', alpha=0.7)
        
        self.ax_compare.set_xticks(x)
        self.ax_compare.set_xticklabels(['X轴', 'Y轴', 'Z轴'])
        self.ax_compare.legend()
        
        return []
    
    def start_animation(self):
        """启动动画"""
        self.animation = FuncAnimation(self.fig, self.update_plots, 
                                     interval=50, blit=False, cache_frame_data=False)
    
    def stop(self):
        """停止可视化"""
        self.is_running = False
    
    def show(self):
        """显示界面"""
        plt.show()


class BluetoothProtocolParser:
    """蓝牙协议解析器类"""
    
    def __init__(self):
        self.buffer = bytearray()
        self.received_packets = []
        
        # 帧率统计
        self.imu_frame_count = 0
        self.imu_fps_start_time = time.time()
        self.imu_current_fps = 0.0
        self.imu_last_fps_update = time.time()
        
    def update_imu_fps(self):
        """更新IMU帧率统计"""
        current_time = time.time()
        if current_time - self.imu_last_fps_update >= 1.0:  # 每秒更新一次
            elapsed_time = current_time - self.imu_fps_start_time
            if elapsed_time > 0:
                self.imu_current_fps = self.imu_frame_count / elapsed_time
                # 重置统计
                self.imu_frame_count = 0
                self.imu_fps_start_time = current_time
                self.imu_last_fps_update = current_time
                
                # 打印帧率信息
                print(f"📊 IMU接收帧率: {self.imu_current_fps:.1f} FPS")
        
    def get_imu_fps(self):
        """获取当前IMU帧率"""
        return self.imu_current_fps
        
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
        crc_received = struct.unpack('>H', packet[-2:])[0]
        
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
        
        while len(self.buffer) >= FIXED_LEN:
            # 查找帧头
            header_pos = -1
            for i in range(len(self.buffer)):
                if self.buffer[i] == FRAME_HEADER:
                    header_pos = i
                    break
            
            if header_pos == -1:
                self.buffer.clear()
                break
            
            # 丢弃帧头前的数据
            if header_pos > 0:
                self.buffer = self.buffer[header_pos:]
            
            # 检查是否有足够的数据读取长度字段
            if len(self.buffer) < 7:
                break
            
            # 读取数据包长度
            length = struct.unpack('<H', self.buffer[5:7])[0]
            
            # 检查是否有完整的数据包
            if len(self.buffer) < length:
                break
            
            # 提取并解析数据包
            packet = bytes(self.buffer[:length])
            parsed_packet = self.parse_packet(packet)
            
            if parsed_packet:
                packets.append(parsed_packet)
            
            # 移除已处理的数据
            self.buffer = self.buffer[length:]
        
        return packets

    # 命令常量
    dt_cmd_imu = 0x02
    
    def get_imu(self, data: bytes, estimator=None, data_queue=None):
        """处理IMU数据"""
        # 更新IMU帧率统计
        self.imu_frame_count += 1
        self.update_imu_fps()
        
        if len(data) == 36:
            # 9个float - 包含磁力计数据
            data2 = struct.unpack('<9f', data)
            gyro_x, gyro_y, gyro_z = data2[0:3]
            acc_x, acc_y, acc_z = data2[3:6]
            mag_x, mag_y, mag_z = data2[6:9]
            
            # 如果有姿态解算器，计算姿态
            if estimator:
                gyro = np.array([gyro_x, gyro_y, gyro_z])
                accel = np.array([acc_x, acc_y, acc_z])
                mag = np.array([mag_x, mag_y, mag_z])
                
                # 计算姿态
                angles = estimator.update_imu(gyro, accel)
                print(f"9轴IMU: Gyro({gyro_x:.1f},{gyro_y:.1f},{gyro_z:.1f})°/s | Accel({acc_x:.1f},{acc_y:.1f},{acc_z:.1f})m/s² | Mag({mag_x:.1f},{mag_y:.1f},{mag_z:.1f})μT | Angles Roll:{angles[0]:.1f}° Pitch:{angles[1]:.1f}° Yaw:{angles[2]:.1f}° | FPS:{self.imu_current_fps:.1f}")
                
                # 如果有数据队列，发送数据到GUI（包含磁力计数据）
                if data_queue:
                    try:
                        data_queue.put_nowait((angles.copy(), accel.copy(), gyro.copy(), mag.copy()))
                    except queue.Full:
                        pass  # 队列满了就丢弃数据
            return
        elif len(data) == 24:
            # 6个float - 6轴IMU（无磁力计）
            data2 = struct.unpack('<6f', data)
            gyro_x, gyro_y, gyro_z = data2[0:3]
            acc_x, acc_y, acc_z = data2[3:6]
            
            # 如果有姿态解算器，计算姿态
            if estimator:
                gyro = np.array([gyro_x, gyro_y, gyro_z])
                accel = np.array([acc_x, acc_y, acc_z])
                mag = np.array([0, 0, 0])  # 6轴IMU无磁力计数据
                
                # 计算姿态
                angles = estimator.update_imu(gyro, accel)
                print(f"6轴IMU: Gyro({gyro_x:.1f},{gyro_y:.1f},{gyro_z:.1f})°/s | Accel({acc_x:.1f},{acc_y:.1f},{acc_z:.1f})m/s² | Angles Roll:{angles[0]:.1f}° Pitch:{angles[1]:.1f}° Yaw:{angles[2]:.1f}° | FPS:{self.imu_current_fps:.1f}")
                
                # 如果有数据队列，发送数据到GUI（磁力计数据为0）
                if data_queue:
                    try:
                        data_queue.put_nowait((angles.copy(), accel.copy(), gyro.copy(), mag.copy()))
                    except queue.Full:
                        pass  # 队列满了就丢弃数据
            return
        else:
            print(f"未知IMU数据长度: {len(data)}")
        return

    def get_cmd_name(self, cmd: int) -> str:
        """获取命令名称"""
        cmd_names = {
            0x0002: "IMU数据",
        }
        return cmd_names.get(cmd, f"未知命令({cmd:04X})")
    
    def print_packet(self, packet: Dict, estimator=None, data_queue=None):
        """打印数据包信息"""
        if packet['data']:
            # 处理IMU数据
            if packet['cmd'] == self.dt_cmd_imu:
                self.get_imu(packet['data'], estimator, data_queue)


class BluetoothWorker:
    """蓝牙工作线程 - 后台线程运行"""
    
    def __init__(self, data_queue):
        self.data_queue = data_queue
        self.parser = BluetoothProtocolParser()
        self.attitude_estimator = IMUAttitudeEstimator()
        self.client = None
        self.is_connected = False
        self.should_run = False
        
    async def scan_devices(self, duration: float = 10.0) -> List[BLEDevice]:
        """扫描蓝牙设备"""
        print(f"扫描蓝牙设备 {duration} 秒...")
        
        try:
            devices = await BleakScanner.discover(timeout=duration)
        except Exception as e:
            print(f"扫描失败: {e}")
            return []
        
        print(f"发现 {len(devices)} 个设备:")
        originflow_devices = []
        
        for i, device in enumerate(devices):
            device_info = f"  {i+1}. {device.name or 'Unknown'}"
            if device.address:
                device_info += f" ({device.address})"
            print(device_info)
            
            # 查找OriginFlow设备
            if device.name and ("OriginFlow" in device.name):
                originflow_devices.append(device)
        
        return originflow_devices
    
    async def connect_device(self, device: BLEDevice) -> bool:
        """连接到指定设备"""
        print(f"\n尝试连接到设备: {device.name} ({device.address})")
        
        try:
            self.client = BleakClient(device)
            await self.client.connect()
            
            if self.client.is_connected:
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
                # 处理协议数据
                packets = self.parser.process_data(data)
                for packet in packets:
                    self.parser.print_packet(packet, self.attitude_estimator, self.data_queue)
            
            await self.client.start_notify(characteristic.uuid, notification_handler)
            print("✓ 通知已启用")
            
        except Exception as e:
            print(f"启用通知失败: {e}")
    
    async def disconnect(self):
        """断开连接"""
        if self.client and self.is_connected:
            await self.client.disconnect()
            self.is_connected = False
            print("设备已断开连接")
    
    async def run_bluetooth(self):
        """运行蓝牙任务"""
        try:
            # 扫描设备
            devices = await self.scan_devices(15.0)
            
            if not devices:
                print("未找到OriginFlow设备")
                return
            else:
                print(f"\n找到 {len(devices)} 个OriginFlow设备:")
                for i, device in enumerate(devices):
                    print(f"  {i+1}. {device.name} ({device.address})")
                
                # 连接第一个设备
                print(f"\n自动连接到第一个设备: {devices[0].name}")
                if await self.connect_device(devices[0]):
                    print("✓ 设备连接成功，开始接收IMU数据")
                    
                    # 保持连接
                    while self.should_run and self.is_connected:
                        await asyncio.sleep(0.1)
                else:
                    print("✗ 设备连接失败")
        except Exception as e:
            print(f"蓝牙任务错误: {e}")
        finally:
            await self.disconnect()
    
    def start(self):
        """启动蓝牙线程"""
        self.should_run = True
        self.thread = threading.Thread(target=self._run_thread, daemon=True)
        self.thread.start()
    
    def _run_thread(self):
        """线程运行函数"""
        # 创建新的事件循环
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            loop.run_until_complete(self.run_bluetooth())
        except Exception as e:
            print(f"蓝牙线程错误: {e}")
        finally:
            loop.close()
    
    def stop(self):
        """停止蓝牙线程"""
        self.should_run = False
        if self.client and self.is_connected:
            # 创建临时事件循环来断开连接
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.disconnect())
            loop.close()


def main():
    """主函数 - GUI主线程"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='9轴IMU姿态实时显示 - 支持蓝牙和串口')
    parser.add_argument('--port', '-p', type=str, help='串口号 (例如: COM3 或 /dev/ttyUSB0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='波特率 (默认: 115200)')
    parser.add_argument('--bluetooth', action='store_true', help='使用蓝牙连接 (默认使用串口)')
    
    args = parser.parse_args()
    
    print("OriginFlow 手环IMU姿态实时显示 - 支持串口和蓝牙")
    print("="*60)
    
    # 创建数据队列
    data_queue = queue.Queue(maxsize=100)
    
    # 创建共享的协议解析器（用于帧率统计）
    shared_parser = BluetoothProtocolParser()
    
    # 根据参数选择通信方式
    if args.bluetooth or not args.port:
        print("使用蓝牙连接模式")
        # 创建并启动蓝牙工作线程
        bluetooth_worker = BluetoothWorker(data_queue)
        bluetooth_worker.start()
        worker = bluetooth_worker
        mode = "蓝牙"
    else:
        print(f"使用串口连接模式: {args.port} @ {args.baudrate} bps")
        # 创建并启动串口工作线程，传递共享解析器
        serial_worker = SerialWorker(data_queue, args.port, args.baudrate, shared_parser)
        if not serial_worker.start():
            print("串口启动失败，程序退出")
            return
        worker = serial_worker
        mode = "串口"
    
    # 创建并启动GUI，传递共享解析器
    visualizer = AttitudeVisualizer(data_queue)
    visualizer.parser = shared_parser  # 设置共享解析器
    visualizer.start_animation()
    
    print(f"✓ GUI界面已启动")
    print(f"✓ {mode}功能在后台运行")
    print("按Ctrl+C退出程序")
    
    try:
        # 显示GUI界面
        visualizer.show()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        # 清理资源
        visualizer.stop()
        worker.stop()
        print("✓ 程序已退出")


if __name__ == "__main__":
    # 检查必要的库是否安装
    missing_libraries = []
    
    # 检查串口库
    try:
        import serial
    except ImportError:
        missing_libraries.append("pyserial")
    
    # 检查蓝牙库（如果使用蓝牙）
    try:
        import bleak
    except ImportError:
        # 只有在需要蓝牙时才检查
        if '--bluetooth' in sys.argv or '-p' not in sys.argv:
            missing_libraries.append("bleak")
    
    # 检查其他必需库
    try:
        import numpy
    except ImportError:
        missing_libraries.append("numpy")
    
    try:
        import matplotlib
    except ImportError:
        missing_libraries.append("matplotlib")
    
    if missing_libraries:
        print("错误: 需要安装以下库:")
        for lib in missing_libraries:
            print(f"  - {lib}")
        print("请运行: pip install " + " ".join(missing_libraries))
        exit(1)
    
    # 运行主程序
    main()
