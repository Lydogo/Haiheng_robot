#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Author: Mingkai Qiu, Junkai Ma

"""
ROS节点: 扫描CANalyst-II设备并将结果上传到ROS参数服务器
"""

import rospy
from CANalyst2_canbus_handler import CanbusHandler

def scan_and_upload():
    rospy.init_node("scan_can_devices_node", anonymous=False)
    rospy.loginfo(" 正在扫描连接的 CANalyst-II 设备...")

    try:
        devices = CanbusHandler.scan_devices()

        if not devices:
            rospy.logwarn(" 未检测到任何 CANalyst-II 设备，请检查连接。")
            rospy.set_param("/canalyst2/devices", [])
            return

        rospy.loginfo(f" 检测到 {len(devices)} 个 CAN 设备：")
        for dev in devices:
            rospy.loginfo(f"  - Index: {dev['device_index']}, SN: {dev['serial_number']}, Type: {dev['hardware_type']}")

        # 上传到ROS参数服务器
        rospy.set_param("/canalyst2/devices", devices)
        rospy.loginfo(" 已将设备信息上传到参数服务器：/canalyst2/devices")

    except Exception as e:
        rospy.logerr(f" 扫描CAN设备时出错: {e}")
        rospy.set_param("/canalyst2/devices", [])
    finally:
        rospy.loginfo(" 节点执行完毕，退出。")

if __name__ == "__main__":
    scan_and_upload()
