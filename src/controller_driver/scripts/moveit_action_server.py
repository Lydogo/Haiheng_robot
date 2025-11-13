#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import threading
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory
from arm_ctrl import ArmController
from typing import List, Dict, Optional

class ArmTrajectoryActionServer:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('arm_trajectory_action_server')
        
        # 读取参数
        self.side_name = rospy.get_param('~side_name', 'RIGHT_ARM')
        self.can_channel = rospy.get_param('~can_channel', 0)
        self.baud_rate = rospy.get_param('~baud_rate', 1000)
        
        # 创建执行结果对象
        self.result = FollowJointTrajectoryResult()
        
        # 初始化机械臂控制器
        try:
            self.arm_controller = ArmController(
                side_name=self.side_name,
                can_channel=self.can_channel,
                baud_rate=self.baud_rate,
                auto_enable=False
            )
        except Exception as e:
            rospy.logerr(f"Failed to initialize ArmController: {e}")
            raise
        
        # 创建Action Server
        self.action_server = actionlib.SimpleActionServer(
            'follow_joint_trajectory',
            FollowJointTrajectoryAction,
            execute_cb=self.execute_trajectory_cb,
            auto_start=False
        )
        
        # 用于轨迹执行的控制变量
        self.execution_thread: Optional[threading.Thread] = None
        self.stop_execution = threading.Event()
        self.trajectory_lock = threading.Lock()
        
        # 启动Action Server
        self.action_server.start()
        rospy.loginfo("Arm trajectory action server is ready.")

    def execute_trajectory_cb(self, goal):
        """处理轨迹执行请求的回调函数"""
        rospy.loginfo("Received new trajectory execution request")
        
        # 检查轨迹是否为空
        if not goal.trajectory.points:
            rospy.logwarn("Received empty trajectory")
            self.result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self.action_server.set_aborted(self.result)
            return
            
        # 停止当前正在执行的轨迹
        self.stop_current_execution()
        
        # 创建新的执行线程
        self.stop_execution.clear()
        self.execution_thread = threading.Thread(
            target=self.execute_trajectory,
            args=(goal.trajectory,)
        )
        self.execution_thread.start()

    def stop_current_execution(self):
        """停止当前轨迹执行"""
        if self.execution_thread and self.execution_thread.is_alive():
            rospy.loginfo("Stopping current trajectory execution...")
            self.stop_execution.set()
            self.execution_thread.join()
            rospy.loginfo("Previous trajectory execution stopped.")

    def execute_trajectory(self, trajectory: JointTrajectory):
        """执行轨迹的具体实现"""
        try:
            # 获取关节名称到电机ID的映射
            joint_to_motor_map = self.get_joint_to_motor_map(trajectory.joint_names)
            
            # 使能所有相关电机
            motor_ids = list(joint_to_motor_map.values())
            self.arm_controller.enable_motors(motor_ids)
            
            # 遍历轨迹点
            for point_idx, point in enumerate(trajectory.points):
                if self.stop_execution.is_set() or rospy.is_shutdown():
                    rospy.loginfo("Trajectory execution interrupted")
                    self.handle_execution_result(success=False)
                    return
                
                # 创建电机位置映射
                motor_positions = {}
                for joint_idx, joint_name in enumerate(trajectory.joint_names):
                    if joint_name in joint_to_motor_map:
                        motor_id = joint_to_motor_map[joint_name]
                        position = point.positions[joint_idx]
                        # 这里可能需要添加单位转换
                        motor_positions[motor_id] = self.convert_to_motor_units(position)
                
                # 执行运动
                with self.trajectory_lock:
                    self.arm_controller.set_specific_motors_position(
                        motor_positions,
                        is_check_move=True
                    )
                
                # 等待指定的时间间隔
                if point_idx < len(trajectory.points) - 1:
                    next_time = trajectory.points[point_idx + 1].time_from_start
                    current_time = point.time_from_start
                    wait_time = (next_time - current_time).to_sec()
                    rospy.sleep(max(0.0, wait_time))
            
            self.handle_execution_result(success=True)
            
        except Exception as e:
            rospy.logerr(f"Error during trajectory execution: {e}")
            self.handle_execution_result(success=False, error_msg=str(e))

    def handle_execution_result(self, success: bool, error_msg: str = ""):
        """处理轨迹执行结果"""
        if success:
            rospy.loginfo("Trajectory execution completed successfully")
            self.result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self.action_server.set_succeeded(self.result)
        else:
            rospy.logerr(f"Trajectory execution failed: {error_msg}")
            self.result.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
            self.result.error_string = error_msg
            self.action_server.set_aborted(self.result)

    def get_joint_to_motor_map(self, joint_names: List[str]) -> Dict[str, int]:
        """获取关节名称到电机ID的映射"""
        # 这里需要根据你的具体配置实现映射关系
        # 示例实现：
        joint_motor_map = {
            'joint1': 1,
            'joint2': 2,
            'joint3': 3,
            'joint4': 4,
            'joint5': 5,
            'joint6': 6,
            'joint7': 7,
        }
        return {name: joint_motor_map[name] for name in joint_names if name in joint_motor_map}

    def convert_to_motor_units(self, position: float) -> int:
        """将弧度单位转换为电机编码器单位"""
        # 这里需要根据你的电机特性实现单位转换
        # 示例实现：
        ENCODER_COUNTS_PER_REVOLUTION = 8192  # 根据实际电机参数调整
        motor_position = int(position * ENCODER_COUNTS_PER_REVOLUTION / (2 * 3.14159))
        return motor_position

    def shutdown(self):
        """清理资源"""
        self.stop_current_execution()
        if hasattr(self, 'arm_controller'):
            del self.arm_controller

if __name__ == '__main__':
    try:
        server = ArmTrajectoryActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'server' in locals():
            server.shutdown()