#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Arm MoveIt Action Server Node
控制多个 CAN 通道下的机械臂，并提供 MoveIt Action 接口。
"""
###打印当前工作目录###
import os
print("Current working directory:", os.getcwd())
import math
import rospy
import threading
import actionlib
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from controller_driver.msg import *

# 导入你已有的控制类
from motor_driver_pkg.arm_ctrl import ArmController   # 含多电机并行控制能力
#from motor_driver_pkg.motor_controller import MotorController  # 提供电流/错误读取等底层接口
#from src.motor_driver_pkg.arm_ctrl import ArmController   # 含多电机并行控制能力
# from src.motor_driver_pkg.motor_controller import MotorController  # 提供电流/错误读取等底层接口


# # 弧度与编码器计数转换函数
# def rad_to_cnt(rad):
#     # 示例转换函数（实际请根据编码器参数调整）
#     return int(rad / 0.001)  # 假设每 count = 0.001 rad
# # 编码器技术与弧度转换函数
# def cnt_to_rad(cnt):
#     # 示例转换函数（实际请根据编码器参数调整）
#     return cnt * 0.001  # 假设每 count = 0.001 rad
# 通过配置文件加载机械臂关节对应的电机 ID 列表

# ============================================================
#                MultiArmMoveItActionServer 主类
# ============================================================

class MultiArmMoveItActionServer:
    def __init__(self):
        rospy.init_node("multi_arm_moveit_action_server", anonymous=False)

        # ───────────── 参数加载 ─────────────
        self.arm_names = rospy.get_param("~arm_names", ["waist","left_arm", "right_arm","head"])
        # self.joint_names = rospy.get_param("~joint_names", [])
        self.config_file = rospy.get_param("~config_file", "config/arm_bind.yaml")
        self.current_threshold_ma = rospy.get_param("~current_threshold_ma", 15000)
        self.publish_rate_hz = rospy.get_param("~publish_rate", 10)
        self.monitor_rate_hz = rospy.get_param("~monitor_rate", 1)

        # ───────────── 初始化 ArmController 实例 ─────────────
        #TODO 初始化一个实例就够了，把要控制的arm都放在这个实例里控制

        self.arm=ArmController(side_names=self.arm_names, config_file=self.config_file)
        self.waist_break_on_arrival()  # 初始化时让腰部电机进入制动状态

        # ───────────── 共享状态 ─────────────
        self.state_lock = threading.Lock()
        self.current_positions = {}
        self.current_velocities = {}
        self.error_flag = False
        #线程锁，这几个值是多线程共享的，state_lock作用是保护这些共享变量的读写安全

        # ───────────── ROS 通信接口 ─────────────
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.diagnostics_pub = rospy.Publisher("/motor_diagnostics", DiagnosticArray, queue_size=5)
        #腿部服务
        self.legs_action_server = actionlib.SimpleActionServer(
            "/legs_controller/leg_move",
            LegMoveAction,
            execute_cb=self.legs_break_on_arrival_cb,
            auto_start=False
        )
        threading.Thread(target=self.legs_action_server.start, daemon=True).start()
        rospy.loginfo("[ACTION SERVER] /legs_controller/leg_move ready.")

        # ——————————为每个move_group创建独立的 action server———————        

        self.action_servers = {}
        for arm_name in self.arm_names:
            action_name = f"/{arm_name}_controller/follow_joint_trajectory"
            server = actionlib.SimpleActionServer(
                action_name,
                FollowJointTrajectoryAction,
                execute_cb=lambda goal, arm=arm_name: self.execute_cb(goal, arm),
                ###actionlib 的回调函数只接受一个参数 goal，但我们需要知道这个回调是属于哪个机械臂的，所以用 lambda 包装一下，让它既能接收 goal，又能知道是哪个 arm_name####
                
                auto_start=False
            )
            self.action_servers[arm_name] = server

            # 独立线程启动 action server
            threading.Thread(target=server.start, daemon=True).start()
            rospy.loginfo(f"[ACTION SERVER] {action_name} ready.")


        # ───────────── 启动后台线程 ────────────
        threading.Thread(target=self._joint_state_publisher, daemon=True).start()
        #self._joint_state_publisher()  # 主线程运行 Joint State 发布函数
        # threading.Thread(target=self._monitor_worker, daemon=True).start()


        rospy.loginfo("[SYSTEM] Multi-Arm MoveIt Action Server running.")

        # ───────────── 注册 shutdown 回调 ─────────────
        rospy.on_shutdown(self._shutdown_cleanup)
        rospy.loginfo("[SYSTEM] Shutdown handler registered.")

        rospy.spin()

    def waist_break_on_arrival(self):

        # motor_id= self.arm.joint_name_to_motor_id.get("WAIST_R", None)
        motor_ids=[13,15]
        if motor_ids is not None:
            commands = [(motor_id,'BRKTO',0) for motor_id in motor_ids]
            self.arm.set_async_move(commands)

    def legs_break_on_arrival_cb(self,goal):
        """
        腿部电机的server回调服务，传入动作参数
        """
        print("Received leg move goal:", goal.target_positions)
        try:
            with self.state_lock:

                motor_ids= self.arm.ARM_CONFIG["legs"]["motor_ids"]
                if motor_ids is not None:
                    commands = [(motor_id,'BRKTO',goal.target_positions[i]) for motor_id,i in zip(motor_ids,range(len(motor_ids)))]
                    self.arm.set_async_move(commands)
        except Exception as e:
            rospy.logerr(f"[LEGS ACTION] Failed to send leg positions: {e}")
            res=LegMoveResult()
            res.success=False
            res.message=str(e)
            self.legs_action_server.set_aborted(res)
            return
        res=LegMoveResult()
        res.success=True
        res.message="Legs moved successfully."
        self.legs_action_server.set_succeeded(res)

    # ============================================================
    #                Joint State 发布线程
    # ============================================================
    # joint_states消息示例：
    # ---
    #     header: 
    #   seq: 10262
    #   stamp: 
    #     secs: 1761033517
    #     nsecs: 414884567
    #   frame_id: ''
    # name: 
    #   - WAIST_R
    #   - WAIST_Y
    #   - WAIST_P
    #   - R_SHOULDER_P
    #   - R_SHOULDER_R
    #   - R_SHOULDER_Y
    #   - R_ELBOW_Y
    #   - R_WRIST_P
    #   - R_WRIST_Y
    #   - R_WRIST_R
    #   - L_SHOULDER_P
    #   - L_SHOULDER_R
    #   - L_SHOULDER_Y
    #   - L_ELBOW_Y
    #   - L_WRIST_P
    #   - L_WRIST_Y
    #   - L_WRIST_R
    #   - NECK_Y
    #   - NECK_P
    #   - NECK_R
    # position: [0.0, 0.0, 0.0, -0.7634958393548881, 1.3619239295405314, 2.612671641606576, 1.886643603331932, -2.4280815190133826, -0.8568889869109686, 0.08956826518391328, -5.851699505001307e-07, 1.5381025744320358, 6.655037207528949e-05, 2.438206884544343e-05, 1.1454376531764866e-05, 7.217745948582889e-05, -7.967071095481516e-06, 0.0, 0.0, 0.0]
    # velocity: []
    # effort: []
    # ---



    # TODO: 根据motor_id_list与joint_names的对应关系，按照joint_state信息示例的顺序和名称来发布话题。
    def _joint_state_publisher(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            #state_lock保护current_positions的读写
            with self.state_lock:

                motor_positions = self.arm.get_target_positions()###get到的positions是按照self.arm.motor_id_list顺序的###
                #如果返回了有None的列表或者列表长度不对，就跳过发布
                if None in motor_positions or len(motor_positions) != len(self.arm.joint_names_list):
                    rospy.logwarn(f"[JOINT STATE] Invalid motor positions read: {motor_positions}")
                    rate.sleep()
                    continue

                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = self.arm.joint_names_list
                msg.position = [self._cnt_to_rad(p) for p in motor_positions]
                msg.velocity = [0.0] * len(msg.name)
                self.current_positions = dict(zip(msg.name,msg.position))###注意这里current_positions是一个字典，joint_name: position_rad###

            self.joint_state_pub.publish(msg)
            rate.sleep()

    # ============================================================
    #                电机监控线程
    # ============================================================
    def _monitor_worker(self):
        rate = rospy.Rate(self.monitor_rate_hz)
        while not rospy.is_shutdown():
            for motor_id, motor in self.arm.motors.items():
                try:
                    current_ma = motor.get_current_current()
                    err_status = motor.get_error_status()
                    if current_ma and abs(current_ma) > self.current_threshold_ma:
                        self._handle_error(f"{motor_id} overcurrent {current_ma}mA")
                    if err_status and err_status != 0:
                        self._handle_error(f"{motor_id} error=0x{err_status:08X}")
                except Exception as e:
                    rospy.logwarn(f"[MONITOR]{motor_id} read failed: {e}")
            rate.sleep()

    # ============================================================
    #                Action Server 执行回调
    # ============================================================
    #/follow_joint_trajectory/goal 消息示例：
    # ---
    #     header: 
    #   seq: 1
    #   stamp: 
    #     secs: 1761036800
    #     nsecs: 899251683
    #   frame_id: ''
    # goal_id: 
    #   stamp: 
    #     secs: 1761036800
    #     nsecs: 899252674
    #   id: "/move_group-2-1761036800.899252674"
    # goal: 
    #   trajectory: 
    #     header: 
    #       seq: 0
    #       stamp: 
    #         secs: 0
    #         nsecs:         0
    #       frame_id: "BASE_S"
    #     joint_names: 
    #       - L_ELBOW_Y
    #       - L_SHOULDER_P
    #       - L_SHOULDER_R
    #       - L_SHOULDER_Y
    #       - L_WRIST_P
    #       - L_WRIST_R
    #       - L_WRIST_Y
    #     points: 
    #       - 
    #         positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #         velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #         accelerations: [0.0, -0.07999853998319502, 0.0, 0.0, 0.0, 0.0, 0.0]
    #         effort: []
    #         time_from_start: 
    #           secs: 0
    #           nsecs:         0
    # ---
    # TODO: 根据实际机械臂的关节命名和顺序，调整arm.set_target_positions(point.positions)电机id和goal.trajectory.points关节顺序的对应关系
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
        
        #按照self.config里的joint_names顺序来排列goal.trajectory.points里的positions
        # joint_name_order = [self.arm.config[arm_name]['joint_names']]
        # joint_name_order.extend(self.arm.config[arm_name]['joint_names'])
        # name_to_index = {name: idx for idx, name in enumerate(goal.trajectory.joint_names)}
        #字典推导 {name: idx for idx, name in ...} 把每个关节名 name 作为键、对应的索引 idx 作为值，生成一个 name -> index 的映射 name_to_index。

        #以上弃用，直接根据self.arm.joint_names_list和self.arm.motor_id_list获取对应电机的motor_id然后直接命令？尝试一下

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
            while rospy.Time.now() < target_time: # and not all(abs(a - d) < 0.01 for a, d in zip(fb.actual.positions, fb.desired.positions)):
                rospy.sleep(0.1)
                if self.error_flag:
                    rospy.logerr(f"[ACTION] Hardware error detected! Abort trajectory for {arm_name}.")
                    res = FollowJointTrajectoryResult()
                    res.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    res.error_string = "Hardware fault detected"
                    self._stop_all()
                    action_server.set_aborted(res)
                    return

            # 只发送目标到当前机械臂
            # 没有判断是否到达目标位置，只是简单地发送命令
            #FIXME: 需要判断是否到达目标位置吗？还是只需要发送命令就行？
            try:
                
                with self.state_lock:
                    for joint_name, position_rad in zip(goal.trajectory.joint_names, point.positions):
                        motor_id = self.arm.joint_name_to_motor_id[joint_name]
                        position_cnt = self._rad_to_cnt(position_rad)
                        self.arm.motors[motor_id].set_target_position(position_cnt)
                    fb = FollowJointTrajectoryFeedback()
                    fb.joint_names = goal.trajectory.joint_names
                    fb.actual.positions = [self.current_positions[jn] for jn in goal.trajectory.joint_names]
                    fb.desired.positions = list(point.positions)
                    action_server.publish_feedback(fb)
                ########## set_target_positions里的电机位置应该与trajectory的goal对应！最好安装ArmController类里YAML设置的motor_idx的顺序来传递##########
            except Exception as e:
                rospy.logerr(f"[ACTION] Failed to send positions for {arm_name}: {e}")
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                res.error_string = str(e)
                action_server.set_aborted(res)
                return arm_name


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
        try:
            self.arm.stop_all_motors()
        except Exception as e:
            rospy.logwarn(f"[STOP] Failed to stop motors: {e}")

    def _shutdown_cleanup(self):
        rospy.loginfo("[SYSTEM] ROS node is shutting down. Stopping all motors and cleaning up...")
        
        try:
            #self.arm.stop_all_motors()
            del self.arm
            rospy.loginfo(f"[CLEANUP] motor stopped and deleted.")
        except Exception as e:
            rospy.logwarn(f"[CLEANUP] Failed to stop motor: {e}")

    @staticmethod
    def _cnt_to_rad(cnt):
        #2π rad对应262144 count
        return cnt * 2*math.pi/262144
    @staticmethod
    def _rad_to_cnt(rad):
        return int(rad * 262144/(2*math.pi))

# ============================================================
#                节点入口
# ============================================================
if __name__ == "__main__":
    try:
        MultiArmMoveItActionServer()
    except rospy.ROSInterruptException:
        pass
