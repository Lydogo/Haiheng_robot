#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Author: Mingkai Qiu, Junkai Ma
"""
Multi-Arm MoveIt Action Server Node
控制多个 CAN 通道下的机械臂，并提供 MoveIt Action 接口。
"""

import rospy
import threading
import actionlib
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult

# 导入你已有的控制类
from arm_ctrl import ArmController  # 含多电机并行控制能力
from motor_controller import MotorController  # 提供电流/错误读取等底层接口


# ============================================================
#                MultiArmMoveItActionServer 主类
# ============================================================
class MultiArmMoveItActionServer:
    def __init__(self):
        rospy.init_node("multi_arm_moveit_action_server", anonymous=False)

        # ───────────── 参数加载 ─────────────
        self.arm_names = rospy.get_param("~arm_names", ["left_arm", "right_arm"])
        self.joint_names = rospy.get_param("~joint_names", [])
        self.current_threshold_ma = rospy.get_param("~current_threshold_ma", 15000)
        self.publish_rate_hz = rospy.get_param("~publish_rate", 50)
        self.monitor_rate_hz = rospy.get_param("~monitor_rate", 10)

        # ───────────── 初始化 ArmController 实例 ─────────────
        self.arms = {}
        for arm_name in self.arm_names:
            try:
                self.arms[arm_name] = ArmController(side_name=arm_name)
                rospy.loginfo(f"[INIT] ArmController initialized: {arm_name}")
            except Exception as e:
                rospy.logerr(f"Failed to init ArmController for {arm_name}: {e}")

        # ───────────── 共享状态 ─────────────
        self.state_lock = threading.Lock()
        self.current_positions = {}
        self.current_velocities = {}
        self.error_flag = False

        # 线程锁，这几个值是多线程共享的，state_lock作用是保护这些共享变量的读写安全

        # ───────────── ROS 通信接口 ─────────────
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.diagnostics_pub = rospy.Publisher("/motor_diagnostics", DiagnosticArray, queue_size=5)

        # 为每个机械臂创建独立的 action server
        self.action_servers = {}
        for arm_name in self.arm_names:
            action_name = f"/{arm_name}_controller/follow_joint_trajectory"

            # actionlib 的回调函数只接受一个参数 goal
            #   但我们需要知道这个回调是属于哪个机械臂的
            #   所以用 lambda 包装一下，让它既能接收 goal，又能知道是哪个 arm_name

            server = actionlib.SimpleActionServer(
                action_name,
                FollowJointTrajectoryAction,
                execute_cb=lambda goal, arm=arm_name: self.execute_cb(goal, arm),
                auto_start=False
            )
            self.action_servers[arm_name] = server

            # 独立线程启动 action server
            threading.Thread(target=server.start, daemon=True).start()
            rospy.loginfo(f"[ACTION SERVER] {action_name} ready.")

        # ───────────── 启动后台线程 ─────────────
        threading.Thread(target=self._joint_state_publisher, daemon=True).start()
        threading.Thread(target=self._monitor_worker, daemon=True).start()

        rospy.loginfo("[SYSTEM] Multi-Arm MoveIt Action Server running.")
        rospy.spin()

    # ============================================================
    #                Joint State 发布线程
    # ============================================================
    def _joint_state_publisher(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            # state_lock保护current_positions的读写
            with self.state_lock:
                all_positions = {}
                for arm_name, arm in self.arms.items():
                    try:
                        motor_positions = arm.get_target_positions()
                        # get_target_positions是获取
                        for motor_id, pos in zip(arm.motor_id_list, motor_positions):
                            all_positions[f"{arm_name}_{motor_id}"] = pos
                    except Exception as e:
                        rospy.logwarn(f"[{arm_name}] failed to read positions: {e}")

                # TODO 发布joint_states的消息要先看看move_group里读取的关节信息顺序和格式，可以网上搜一搜

                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = list(all_positions.keys())
                msg.position = [self._cnt_to_rad(p) for p in all_positions.values()]
                msg.velocity = [0.0] * len(msg.name)
                self.current_positions = all_positions

            self.joint_state_pub.publish(msg)
            rate.sleep()

    # ============================================================
    #                电机监控线程
    # ============================================================
    def _monitor_worker(self):
        rate = rospy.Rate(self.monitor_rate_hz)
        while not rospy.is_shutdown():
            for arm_name, arm in self.arms.items():
                for motor_id, motor in arm.motors.items():
                    try:
                        current_ma = motor.get_current_current()
                        err_status = motor.get_error_status()
                        if current_ma and abs(current_ma) > self.current_threshold_ma:
                            self._handle_error(f"{arm_name}:{motor_id} overcurrent {current_ma}mA")
                        if err_status and err_status != 0:
                            self._handle_error(f"{arm_name}:{motor_id} error=0x{err_status:08X}")
                    except Exception as e:
                        rospy.logwarn(f"[MONITOR] {arm_name}:{motor_id} read failed: {e}")
            rate.sleep()

    # ============================================================
    #                Action Server 执行回调
    # ============================================================
    def execute_cb(self, goal, arm_name):
        rospy.loginfo(f"[ACTION] Received FollowJointTrajectory goal for {arm_name}.")
        action_server = self.action_servers[arm_name]

        # 1. 基础检查
        if not goal.trajectory.points:
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            res.error_string = "Empty trajectory."
            action_server.set_aborted(res)
            return

        # 2. 按时间执行轨迹
        start_time = rospy.Time.now()
        for point_idx, point in enumerate(goal.trajectory.points):
            if action_server.is_preempt_requested():
                rospy.logwarn(f"[ACTION] Goal preempted by client for {arm_name}.")
                self._stop_all()
                action_server.set_preempted()
                return

            # 等待到达该时间点
            target_time = start_time + point.time_from_start
            while rospy.Time.now() < target_time:
                rospy.sleep(0.01)
                if self.error_flag:
                    rospy.logerr(f"[ACTION] Hardware error detected! Abort trajectory for {arm_name}.")
                    res = FollowJointTrajectoryResult()
                    res.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    res.error_string = "Hardware fault detected"
                    self._stop_all()
                    action_server.set_aborted(res)
                    return

            # 只发送目标到当前机械臂
            try:
                arm = self.arms[arm_name]
                arm.set_target_positions(point.positions)
                # TODO
                #   set_target_positions里的电机位置应该与trajectory的goal对应！
                #   最好写个新的set_target_positions获取对应关系

            except Exception as e:
                rospy.logerr(f"[ACTION] Failed to send positions for {arm_name}: {e}")
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                res.error_string = str(e)
                action_server.set_aborted(res)
                return

            # 发布 feedback
            fb = FollowJointTrajectoryFeedback()
            fb.joint_names = goal.trajectory.joint_names
            with self.state_lock:
                fb.actual.positions = [self._cnt_to_rad(p) for p in self.current_positions.values()]
            fb.desired.positions = list(point.positions)
            action_server.publish_feedback(fb)

        # 3. 执行完毕
        res = FollowJointTrajectoryResult()
        res.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        action_server.set_succeeded(res)
        rospy.loginfo(f"[ACTION] Trajectory execution completed successfully for {arm_name}.")

    # ============================================================
    #                辅助函数
    # ============================================================
    def _handle_error(self, msg):
        rospy.logerr(f"[MONITOR ERROR] {msg}")
        self.error_flag = True
        diag = DiagnosticArray()
        d = DiagnosticStatus()
        d.name = "Motor Error"
        d.level = DiagnosticStatus.ERROR
        d.message = msg
        diag.status.append(d)
        diag.header.stamp = rospy.Time.now()
        self.diagnostics_pub.publish(diag)

    def _stop_all(self):
        rospy.logwarn("[SYSTEM] Stopping all motors.")
        for arm in self.arms.values():
            try:
                arm.stop_motors()
            except Exception as e:
                rospy.logwarn(f"[STOP] Failed to stop motors: {e}")

    @staticmethod
    def _cnt_to_rad(cnt):
        # 示例转换函数（实际请根据编码器参数调整）
        return cnt * 0.001  # 假设每 count = 0.001 rad


# ============================================================
#                节点入口
# ============================================================
if __name__ == "__main__":
    try:
        MultiArmMoveItActionServer()
    except rospy.ROSInterruptException:
        pass
