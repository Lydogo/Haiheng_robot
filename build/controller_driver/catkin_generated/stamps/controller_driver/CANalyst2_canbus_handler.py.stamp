# -*- coding: utf-8 -*-
# @Author: Junkai Ma

import ctypes
import platform
from typing import Dict, List, Optional

VCI_USBCAN2 = 4
STATUS_OK = 1

BAUD_RATE_MAP = {
    1000: (0x00, 0x14),  # 1000 kbps
    800: (0x00, 0x16),  # 800 kbps
    500: (0x00, 0x1C),  # 500 kbps
    250: (0x01, 0x1C),  # 250 kbps
    125: (0x03, 0x1C),  # 125 kbps (示例中使用的值)
    100: (0x04, 0x1C),  # 100 kbps
    50: (0x09, 0x1C),  # 50 kbps
}


class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", ctypes.c_uint),
        ("AccMask", ctypes.c_uint),
        ("Reserved", ctypes.c_uint),
        ("Filter", ctypes.c_ubyte),
        ("Timing0", ctypes.c_ubyte),
        ("Timing1", ctypes.c_ubyte),
        ("Mode", ctypes.c_ubyte),
    ]


class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint),
        ("TimeStamp", ctypes.c_uint),
        ("TimeFlag", ctypes.c_ubyte),
        ("SendType", ctypes.c_ubyte),
        ("RemoteFlag", ctypes.c_ubyte),
        ("ExternFlag", ctypes.c_ubyte),
        ("DataLen", ctypes.c_ubyte),
        ("Data", ctypes.c_ubyte * 8),
        ("Reserved", ctypes.c_ubyte * 3),
    ]


class VCI_BOARD_INFO(ctypes.Structure):
    _fields_ = [
        ("HardwareVersion", ctypes.c_ushort),
        ("FirmwareVersion", ctypes.c_ushort),
        ("DriverVersion", ctypes.c_ushort),
        ("InterfaceVersion", ctypes.c_ushort),
        ("InterruptNumber", ctypes.c_ushort),
        ("CANChannelCount", ctypes.c_ubyte),
        ("SerialNumber", ctypes.c_char * 20),
        ("HardwareType", ctypes.c_char * 40),
        ("Reserved", ctypes.c_ushort * 4)
    ]


class CanbusHandler:
    """
    封装了与CANalyst-II分析仪通信的底层细节。
    """
    _instance_count = 0

    def __init__(self,
                 can_channel: int = 0,
                 device_index: int = 0,
                 serial_number: Optional[str] = None,
                 baud_rate: int = 1000,):
        """
        初始化并打开CAN设备。

        :param can_channel: 要使用的CAN通道 (0 或 1)。
        :param baud_rate: 波特率 (kbps)。
        :param device_index: 设备索引号 (当 serial_number 为 None 时使用)。
        :param serial_number: (推荐) 要连接的设备的序列号。如果提供，将忽略 device_index。
        """
        if CanbusHandler._instance_count > 0:
            # 确保在多实例场景下不会重复加载DLL
            pass

        # 根据操作系统加载对应的库文件
        system = platform.system()
        if system == "Windows":
            self.can_dll = ctypes.windll.LoadLibrary('./ControlCAN.dll')
        elif system == "Linux":
            self.can_dll = ctypes.cdll.LoadLibrary('./libs/libcontrolcan.so')
        else:
            raise OSError(f"Unsupported operating system: {system}")

        self.can_channel = can_channel
        self.device_type = VCI_USBCAN2
        self.final_device_index = -1

        # !! 新增: 通过序列号查找设备索引
        if serial_number:
            devices = self.scan_devices()
            found = False
            for device in devices:
                if device['serial_number'] == serial_number:
                    self.final_device_index = device['device_index']
                    found = True
                    break
            if not found:
                raise IOError(f"找不到序列号为 '{serial_number}' 的CAN设备。")
        else:
            self.final_device_index = device_index

        # 波特率映射表
        # baud_map = {
        #     1000: (0x00, 0x14), 500: (0x00, 0x1C), 250: (0x01, 0x1C),
        #     125: (0x03, 0x1C), 100: (0x04, 0x1C), 50: (0x09, 0x1C)
        # }
        if baud_rate not in BAUD_RATE_MAP:
            raise ValueError(f"不支持的波特率: {baud_rate} kbps")

        timing0, timing1 = BAUD_RATE_MAP[baud_rate]
        self.init_config = VCI_INIT_CONFIG(0, 0xFFFFFFFF, 0, 0, timing0, timing1, 0)

        CanbusHandler._instance_count += 1

    def open(self):
        """打开设备并初始化指定的CAN通道"""
        ret = self.can_dll.VCI_OpenDevice(self.device_type, self.final_device_index, 0)
        if ret != STATUS_OK:
            raise IOError(f"打开CAN设备 (Index: {self.final_device_index}) 失败。")

        ret = self.can_dll.VCI_InitCAN(self.device_type, self.final_device_index, self.can_channel,
                                       ctypes.byref(self.init_config))
        if ret != STATUS_OK:
            self.close()
            raise IOError(f"初始化CAN通道 {self.can_channel} 失败。")

        ret = self.can_dll.VCI_StartCAN(self.device_type, self.final_device_index, self.can_channel)
        if ret != STATUS_OK:
            self.close()
            raise IOError(f"启动CAN通道 {self.can_channel} 失败。")

        # 清空缓冲区
        self.can_dll.VCI_ClearBuffer(self.device_type, self.final_device_index, self.can_channel)

    def close(self):
        """关闭CAN设备"""
        self.can_dll.VCI_CloseDevice(self.device_type, self.final_device_index)
        CanbusHandler._instance_count -= 1

    def send(self, frame: VCI_CAN_OBJ):
        """发送一帧CAN数据"""
        ret = self.can_dll.VCI_Transmit(self.device_type, self.final_device_index, self.can_channel,
                                        ctypes.byref(frame), 1)
        if ret != STATUS_OK:
            raise IOError("CAN帧发送失败。")

    def receive(self, buffer_size: int = 100, wait_time_ms: int = 20) -> Optional[List[VCI_CAN_OBJ]]:
        """接收CAN数据"""
        rx_array_type = VCI_CAN_OBJ * buffer_size
        rx_frame_buffer = rx_array_type()

        ret = self.can_dll.VCI_Receive(self.device_type, self.final_device_index, self.can_channel,
                                       ctypes.byref(rx_frame_buffer), buffer_size, wait_time_ms)
        if ret > 0:
            return [rx_frame_buffer[i] for i in range(ret)]
        return None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @staticmethod
    def scan_devices() -> List[Dict]:
        """
        扫描所有连接的USB-CAN分析仪设备。静态方法。用于直接绑定设备uuid。
        :return: 一个包含设备信息的字典列表。
        """
        system = platform.system()
        if system == "Windows":
            dll = ctypes.windll.LoadLibrary('./ControlCAN.dll')
        elif system == "Linux":
            dll = ctypes.cdll.LoadLibrary('./libs/libcontrolcan.so')
        else:
            raise OSError(f"Unsupported operating system: {system}")

        # 准备缓冲区并调用底层函数
        info_array_type = VCI_BOARD_INFO * 50
        info_buffer = info_array_type()

        try:
            num_devices = dll.VCI_FindUsbDevice2(ctypes.byref(info_buffer))
        except Exception as e:
            raise RuntimeError(f"调用 VCI_FindUsbDevice2 失败: {e}")

        devices: List[Dict] = []
        if num_devices and num_devices > 0:
            for i in range(num_devices):
                info = info_buffer[i]
                # 安全解码 C 固定长度字符串（截断到第一个 NUL）
                serial_bytes = info.SerialNumber.split(b'\x00', 1)[0]
                hw_bytes = info.HardwareType.split(b'\x00', 1)[0]
                serial_number = serial_bytes.decode('ascii', errors='ignore')
                hardware_type = hw_bytes.decode('ascii', errors='ignore')

                # 某些结构体定义可能不包含完全相同字段，使用 getattr 防护
                can_channels = getattr(info, 'CANChannelCount', None)

                devices.append({
                    'device_index': i,
                    'serial_number': serial_number,
                    'hardware_type': hardware_type,
                    'can_channels': can_channels
                })

        return devices
