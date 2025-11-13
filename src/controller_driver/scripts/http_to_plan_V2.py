#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
import tf.transformations as tf_trans
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import moveit_msgs.msg
import rospy
import actionlib
from flask import Flask, request, jsonify
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState, Constraints
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import OrientationConstraint
from controller_driver.msg import *

app = Flask(__name__)

# 全局变量
_robot = None
_scene = None
_current_joint_state = None
_ik_service = None
_move_groups = {}
_initialized = False
_legmove_client = None 
_TF_BUFFER = None
_TF_LISTENER = None
# K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
_CAMERA_K = np.array([
    [608.3013305664062, 0.0, 324.0627746582031],
    [0.0, 608.1605224609375, 248.2197723388672],
    [0.0, 0.0, 1.0]
])

"""
初始化相关函数
"""
def initialize_moveit():
    global _robot, _scene,  _initialized, _legmove_client
    if _initialized:
        return True
    
    try:
        # 初始化moveit_commander和rospy节点
        moveit_commander.roscpp_initialize(sys.argv)
        # 实例化RobotCommander对象
        _robot = moveit_commander.RobotCommander()
        # 实例化PlanningSceneInterface对象
        _scene = moveit_commander.PlanningSceneInterface() 
        # 初始化腿部动作客户端
        _legmove_client = actionlib.SimpleActionClient("/legs_controller/leg_move", LegMoveAction)
        # 订阅关节状态话题
        rospy.Subscriber('/joint_states', JointState, joint_state_callback)
        rospy.sleep(0.5)
        _initialized = True
        print("MoveIt initialized successfully!")
        return True
        
    except Exception as e:
        print(f"Failed to initialize: {str(e)}")
        return False

def get_move_group(group_name):
    """获取或创建MoveGroupCommander对象，使用缓存避免重复创建"""
    global _move_groups
    if group_name not in _move_groups:
        try:
            move_group = moveit_commander.MoveGroupCommander(group_name)
            # 设置工作空间
            move_group.set_workspace([-1.0, -1.0, -1.0, 1.0, 1.0, 1.0])
            move_group.set_planning_pipeline_id("ompl")
            move_group.set_planner_id("RRT")
            _move_groups[group_name] = move_group
            print(f"MoveGroupCommander for group '{group_name}' created successfully!")
            
        except Exception as e:
            print(f"Failed to create MoveGroupCommander for group '{group_name}': {str(e)}")
            return None
    
    return _move_groups[group_name]

def joint_state_callback(msg):
    """关节状态回调函数"""
    global _current_joint_state
    _current_joint_state = msg
        
def get_end_effector_link(group_name):
    if group_name == "Right_arm":
        return "R_WRIST_R_S"
    elif group_name == "Left_arm":
        return "L_WRIST_R_S"


"""
坐标转换相关函数
"""
#将像素坐标和深度转换为相机坐标系下的三维坐标。
def pixel_to_camera_xyz(depth, u, v, K=_CAMERA_K):  
    if K is None:
        K = _CAMERA_K
    if not isinstance(K, np.ndarray):
        K = np.array(K).reshape(3, 3)

    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])
    zc = float(depth)
    xc = (float(u) - cx) * zc / fx
    yc = (float(v) - cy) * zc / fy
    return {'x': xc, 'y': yc, 'z': zc}

def caculate_target_pose_from_pixel_pose(pixel_x, pixel_y, depth, theta,
                                    camera_frame,
                                    base_frame,
                                    timeout=1.0):
    """
    输入像素(u,v)、深度d与旋转角度theta(单位: degrees)，
    输出在base坐标系下的位置(position)与姿态(orientation)。
    返回(position_dict, orientation_dict)，失败返回(None, None)。
    """
    global _TF_BUFFER, _TF_LISTENER
    cam_xyz = pixel_to_camera_xyz(depth, pixel_x, pixel_y)
    if _TF_BUFFER is None:
        _TF_BUFFER = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        _TF_LISTENER = tf2_ros.TransformListener(_TF_BUFFER)
    try:
        tf_msg = _TF_BUFFER.lookup_transform(
            base_frame,
            camera_frame,
            rospy.Time(0),
            rospy.Duration(timeout)
        )
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        print(f"TF lookup failed ({camera_frame} -> {base_frame}): {str(e)}")
        return None, None

    pose_cam = geometry_msgs.msg.PoseStamped()
    pose_cam.header.frame_id = camera_frame
    pose_cam.header.stamp = rospy.Time(0)
    pose_cam.pose.position.x = float(cam_xyz['x'])
    pose_cam.pose.position.y = float(cam_xyz['y'])
    pose_cam.pose.position.z = float(cam_xyz['z'])
    pose_cam.pose.orientation.x = 0.0
    pose_cam.pose.orientation.y = 0.0
    pose_cam.pose.orientation.z = 0.0
    pose_cam.pose.orientation.w = 1.0
    pose_base = do_transform_pose(pose_cam, tf_msg)

    theta_rad = np.deg2rad(float(theta))
    q_z = tf_trans.quaternion_about_axis(np.deg2rad(90.0), (0.0, 0.0, 1.0))
    q_x = tf_trans.quaternion_about_axis(theta_rad, (1.0, 0.0, 0.0))
    q_total = tf_trans.quaternion_multiply(q_x, q_z)
    target_position = {
        "x": pose_base.pose.position.x,
        "y": pose_base.pose.position.y,
        "z": pose_base.pose.position.z
    }
    target_orientation = {
        "x": q_total[0],
        "y": q_total[1],
        "z": q_total[2],
        "w": q_total[3]
    }
    print(f"Successfully calculated target pose!")
    return target_position, target_orientation

def caculate_pre_pose_from_target_pose(position, orientation, distance=0.1):
    target_position = np.array([
        float(position['x']),
        float(position['y']),
        float(position['z'])
    ])
    quat = [
        float(orientation['x']),
        float(orientation['y']),
        float(orientation['z']),
        float(orientation['w'])
    ]
    quat_norm = np.linalg.norm(quat)
    quat = np.array(quat) / quat_norm
    rotation_matrix = tf_trans.quaternion_matrix(quat)
    y_axis = rotation_matrix[:3, 1]
    direction_vector = y_axis / np.linalg.norm(y_axis)
    # 向远离书架方向运动指定距离
    displacement = direction_vector * distance
    pre_position = target_position + displacement
    
    print(f"Successfully calculated pre pose!")
    pre_position_dict = {
        "x": float(pre_position[0]),
        "y": float(pre_position[1]),
        "z": float(pre_position[2])
    }
    return pre_position_dict, orientation

"""
电机服务相关函数
"""
def done_cb(state, result):
    # 服务端返回的结果：LegMoveResult(success, message)
    if state == actionlib.GoalStatus.SUCCEEDED:
        print(f"腿部电机服务完成: success={result.success}, message={result.message}")
    else:
        success = getattr(result, 'success', False)
        message = getattr(result, 'message', '')
        print(f"腿部电机服务结束，状态码={state}, success={success}, message={message}")

def active_cb():
    print("腿部电机运动服务被激活....")

def fb_cb(fb):
    print("当前进度:%.2f",fb.progress_bar)

def leg_client_start(target_positions, timeout=10.0):
    global _legmove_client
    # 等待动作服务器可用
    if not _legmove_client.wait_for_server(rospy.Duration(5.0)):
        print("Leg move action server not available")
        return False
    # 创建并发送目标
    goal_obj = LegMoveGoal()
    goal_obj.target_positions = target_positions
    # 发送目标并接收回调日志
    _legmove_client.send_goal(goal_obj, done_cb=done_cb, active_cb=active_cb, feedback_cb=fb_cb)
    # 等待结果或超时
    finished = _legmove_client.wait_for_result(rospy.Duration(timeout))
    if not finished:
        print("Leg move timed out; canceling goal")
        _legmove_client.cancel_goal()
        return False
    # 读取结果并判断 success
    result = _legmove_client.get_result()
    if result is None:
        print("Leg move result is None")
        return False
    return bool(getattr(result, 'success', False))



"""
运动规划相关函数
"""
def solve_ik_for_pose(group_name, target_pose, timeout=10, path_constraints=None):
    global _ik_service
    if _ik_service is None:
        rospy.wait_for_service('compute_ik', timeout=10.0)
        _ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    # 创建IK请求
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = group_name
    ik_request.ik_request.pose_stamped.header.frame_id = "BASE_S"
    ik_request.ik_request.pose_stamped.pose = target_pose
    ik_request.ik_request.timeout = rospy.Duration(timeout)
    
    # 设置末端执行器链接名称
    ik_request.ik_request.ik_link_name = get_end_effector_link(group_name)
    # 设置起始状态
    robot_state = RobotState()
    robot_state.joint_state = _current_joint_state
    ik_request.ik_request.robot_state = robot_state
    
    # 如果有路径约束，添加到IK请求中
    if path_constraints is not None:
        ik_request.ik_request.constraints = path_constraints
    print(f"Get current joint state: {robot_state.joint_state.position}")
    # 调用IK服务
    ik_response = _ik_service(ik_request)
    print(f"Group: {group_name}")
    print(f"Target pose: {target_pose}")
    print(f"  End-effector link: {ik_request.ik_request.ik_link_name}")

    if ik_response.error_code.val == ik_response.error_code.SUCCESS:
        joint_values = list(ik_response.solution.joint_state.position)
        joint_names = list(ik_response.solution.joint_state.name)
        print(f"IK solution found!")
        return joint_values, joint_names
    else:
        print(f"IK solver failed with error code: {ik_response.error_code.val}")
        return None, None

def plan_to_joint_target(move_group, joint_values, joint_names):
    try:
        # 只保留属于当前规划组的关节，否则ik逆解会把其他关节也设置为目标
        group_joint_names = move_group.get_active_joints()
        if joint_names and joint_values:
            full_joint_goal = dict(zip(joint_names, joint_values))
            filtered_joint_goal = {}
            for joint_name in group_joint_names:
                if joint_name in full_joint_goal:
                    filtered_joint_goal[joint_name] = full_joint_goal[joint_name]
            move_group.set_joint_value_target(filtered_joint_goal)
        else:
            move_group.set_joint_value_target(joint_values)
        
        move_group.set_planning_time(0.5)
        move_group.set_num_planning_attempts(5)
        move_group.set_max_velocity_scaling_factor(0.8)
        move_group.set_max_acceleration_scaling_factor(0.8)
        # 执行规划和运动
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        return success
        
    except Exception as e:
        print(f"Error in joint space planning: {str(e)}")
        return False

def plan_pose_once(group_name, position, orientation, is_straight_constraint=False):
    move_group = get_move_group(group_name)
    if move_group is None:
        print(f"Invalid group_name: {group_name}")
        return False

    # 使用订阅到的真实关节状态作为起始状态
    if _current_joint_state is not None:
        start_state = move_group.get_current_state()
        start_state.joint_state.header = _current_joint_state.header
        start_state.joint_state.name = _current_joint_state.name
        start_state.joint_state.position = _current_joint_state.position
        start_state.joint_state.velocity = _current_joint_state.velocity
        start_state.joint_state.effort = _current_joint_state.effort
        move_group.set_start_state(start_state)
    else:
        move_group.set_start_state_to_current_state()

    # 创建目标姿态
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = float(position['x'])
    pose_goal.position.y = float(position['y'])
    pose_goal.position.z = float(position['z'])
    pose_goal.orientation.x = float(orientation['x'])
    pose_goal.orientation.y = float(orientation['y'])
    pose_goal.orientation.z = float(orientation['z'])
    pose_goal.orientation.w = float(orientation['w'])

    path_constraints = None
    if is_straight_constraint:
        # 保持目标姿态不变（使用目标姿态作为约束）
        quaternion_stamped = geometry_msgs.msg.QuaternionStamped()
        quaternion_stamped.header.frame_id = "BASE_S"
        quaternion_stamped.quaternion = pose_goal.orientation
        end_effector_link = get_end_effector_link(group_name)
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "BASE_S"
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = quaternion_stamped.quaternion
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        path_constraints = Constraints()
        path_constraints.orientation_constraints.append(orientation_constraint)

    joint_values, joint_names = solve_ik_for_pose(group_name, pose_goal, timeout=20.0, path_constraints=path_constraints)
    if joint_values is None:
        print("IK solver failed to find solution for target pose")
        return False
    return plan_to_joint_target(move_group, joint_values, joint_names)


"""
flask接口函数
"""
@app.route('/leg_move', methods=['POST'])
def leg_move():
    data = request.get_json()
    if data is None:
        return jsonify({
            "success": False,
            "error": "No JSON data provided"
        }), 400
    if 'target_positions' not in data:
        return jsonify({
            "success": False,
            "error": "Missing target_positions"
        }), 400
    
    target_positions = data['target_positions']
    # 调用腿部动作客户端并等待执行结果
    success = leg_client_start(target_positions)
    print("success=",success)
    if success:
        return jsonify({
            "success": True,
            "message": "Leg move command sent successfully",
            "target_positions": target_positions
        }), 200
    else:
        return jsonify({
            "success": False,
            "error": "failed"
        }), 500


@app.route('/get_current_pose_http', methods=['GET'])
def get_current_pose_http():
    if not _initialized:
        return jsonify({
            "success": False,
            "error": "MoveIt interface not initialized"
        }), 503
    
    group_name = request.args.get('group_name')
    if not group_name:
        return jsonify({
            "success": False,
            "error": "Missing required parameter: group_name"
        }), 400
    
    move_group = get_move_group(group_name)
    if move_group is None:
        return jsonify({
            "success": False,
            "error": f"Invalid group_name: {group_name}"
        }), 400
    current_pose_stamped = move_group.get_current_pose()
    if current_pose_stamped is None:
        return jsonify({
            "success": False,
            "error": "Failed to get current pose: pose_stamped is None"
        }), 500
    current_pose = current_pose_stamped.pose
    if current_pose is None:
        return jsonify({
            "success": False,
            "error": "Failed to get current pose: pose is None"
        }), 500
    
    response_data = {
        "success": True,
        "group_name": group_name,
        "pose": {
            "position": {
                "x": round(current_pose.position.x, 6),
                "y": round(current_pose.position.y, 6),
                "z": round(current_pose.position.z, 6)
            },
            "orientation": {
                "x": round(current_pose.orientation.x, 6),
                "y": round(current_pose.orientation.y, 6),
                "z": round(current_pose.orientation.z, 6),
                "w": round(current_pose.orientation.w, 6)
            }
        }
    }
    return jsonify(response_data), 200
        


@app.route('/calculate_pre_position', methods=['POST'])
def calculate_pre_position():
    data = request.get_json()
    if not data:
        return jsonify({
            "success": False,
            "error": "No JSON data provided"
        }), 400
    
    group_name = data.get('group_name', 'Right_arm')
    position = data.get('position', {})
    orientation = data.get('orientation', {})
    distance = float(data.get('distance', 0.1))
    # 调用计算函数
    pre_position,pre_orientation = caculate_pre_pose_from_target_pose(position, orientation, distance)
    if pre_position is None or pre_orientation is None:
        return jsonify({
            "success": False,
            "error": f"Failed to calculate pre_position"
        }), 500
    
    response_data = {
        "success": True,
        "group_name": group_name,
        "pose": {
            "position": {
                "x": round(pre_position['x'], 6),
                "y": round(pre_position['y'], 6),
                "z": round(pre_position['z'], 6)
            },
            "orientation": {
                "x": round(pre_orientation['x'], 6),
                "y": round(pre_orientation['y'], 6),
                "z": round(pre_orientation['z'], 6),
                "w": round(pre_orientation['w'], 6)
            }
        }
    }
    return jsonify(response_data), 200

@app.route('/calculate_target_position_from_pixel', methods=['POST'])
def calculate_target_position_from_pixel():
    data = request.get_json()
    if not data:
        return jsonify({
            "success": False,
            "error": "No JSON data provided"
        }), 400

    group_name = data.get('group_name', 'Right_arm')
    pixel_x = data.get('x')
    pixel_y = data.get('y')
    depth     = data.get('d')
    theta     = data.get('theta', 0.0)
    camera_frame = 'camera_color_link'
    base_frame   = 'BASE_S'

    target_position, target_orientation = caculate_target_pose_from_pixel_pose(
        pixel_x, pixel_y, depth, theta,
        camera_frame, base_frame
    )

    if target_position is None or target_orientation is None:
        return jsonify({
            "success": False,
            "error": "Failed to calculate target pose from pixel"
        }), 500

    response_data = {
        "success": True,
        "group_name": group_name,
        "pose": {
            "position": {
                "x": round(target_position['x'], 6),
                "y": round(target_position['y'], 6),
                "z": round(target_position['z'], 6)
            },
            "orientation": {
                "x": round(target_orientation['x'], 6),
                "y": round(target_orientation['y'], 6),
                "z": round(target_orientation['z'], 6),
                "w": round(target_orientation['w'], 6)
            }
        }
    }
    return jsonify(response_data), 200

@app.route('/plan_to_position', methods=['POST'])
def plan_to_position():
    data = request.get_json()
    if data is None:
        return jsonify({"error": "No JSON data provided"}), 400
    if 'group_name' not in data:
        return jsonify({"error": "Missing group_name"}), 400
    if 'position' not in data:
        return jsonify({"error": "Missing position"}), 400
    if 'orientation' not in data:
        return jsonify({"error": "Missing orientation"}), 400

    group_name = data['group_name']
    position = data['position']
    orientation = data['orientation']
    is_straight_constraint = data.get('is_straight_constraint', False)  # 默认为False

    ok = plan_pose_once(group_name, position, orientation, is_straight_constraint)
    if ok:
        return jsonify({"message": "Success"}), 200
    else:
        return jsonify({"message": "Failed to reach goal position"}), 500


if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('http_moveit_bridge', anonymous=True, disable_signals=True)
    
    # 初始化MoveIt接口
    if not initialize_moveit():
        print("Failed to initialize MoveIt interface")
        sys.exit(1)
    
    # 获取参数
    host = str(rospy.get_param('~host', '0.0.0.0'))
    port = int(rospy.get_param('~port', 5000))
    debug = bool(rospy.get_param('~debug', True))
    print(f"Starting HTTP server on {host}:{port}")
    # 启动Flask应用
    app.run(host=host, port=port, debug=debug, use_reloader=False, threaded=True)
