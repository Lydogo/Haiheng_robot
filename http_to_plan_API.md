# HTTP to Plan API 接口文档

## 概述

`http_to_plan_V2.py` 是一个基于Flask的HTTP服务，提供MoveIt运动规划的Web API接口。该服务允许通过HTTP请求控制机器人手臂的运动规划和执行。此版本是V2版本，对原有功能进行了优化和扩展。

### V2版本主要改进

1. **函数命名优化**: 将 `calculate_out_position` 更名为 `calculate_pre_position`，更准确地反映了其预位置计算功能
2. **相机坐标系更新**: 像素转换功能中，相机坐标系从 `camera_link` 更新为 `camera_color_link`
3. **参数简化**: 在像素转换功能中，`camera_frame` 和 `base_frame` 参数被固定，简化了API调用
4. **错误处理统一**: 所有错误响应都统一使用 `success: false` 和 `error` 字段格式
5. **代码结构优化**: 改进了函数组织和错误处理逻辑

## API接口

### 1. 获取当前位姿

**接口:** `GET /get_current_pose_http`

**描述:** 获取指定规划组末端执行器的当前6维位置数据

**请求参数:**
- `group_name` (query): 动作组名称（必需）

**请求示例:**
```
GET http://localhost:5000/get_current_pose_http?group_name=Right_arm
```

**响应格式:**
```json
{
  "success": true,
  "group_name": "Right_arm",
  "pose": {
    "position": { "x": 0.123, "y": 0.456, "z": 0.789 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  }
}
```

**错误响应:**
```json
{
  "success": false,
  "error": "错误信息"
}
```
- `400`: 缺少group_name参数
- `503`: MoveIt接口未初始化

### 2. 计算预位置

**接口:** `POST /calculate_pre_position`

**描述:** 根据目标位姿计算预位置，用于路径规划和避障

**请求格式:**
```json
{
  "group_name": "Right_arm",
  "position": { "x": 0.3, "y": 0.2, "z": 0.5 },
  "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 },
  "distance": 0.1
}
```

**请求参数:**
- `group_name` (str): 动作组名称（可选，默认 `Right_arm`）
- `position` (dict): 目标位姿位置 {x, y, z}（必需）
- `orientation` (dict): 目标位姿四元数 {x, y, z, w}（必需）
- `distance` (float): 偏移距离（可选，默认0.1米）

**响应格式:**
```json
{
  "success": true,
  "group_name": "Right_arm",
  "pose": {
    "position": { "x": 0.25, "y": 0.15, "z": 0.45 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  }
}
```

**错误响应:**
```json
{
  "success": false,
  "error": "错误信息"
}
```
- `400`: 无JSON数据
- `500`: 内部错误或计算失败

### 3. 运动规划到指定位置

**接口:** `POST /plan_to_position`

**描述:** 接收动作组和末端执行器位置，通过MoveIt Python接口进行运动规划和执行

**请求格式:**
```json
{
    "group_name": "Right_arm",
    "position": {
        "x": 0.3,
        "y": 0.2,
        "z": 0.5
    },
    "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
    },
    "is_straight_constraint": false
}
```

**请求参数:**
- `group_name` (str): 动作组名称（必需）
- `position` (dict): 末端执行器目标位置 {x, y, z}（必需）
- `orientation` (dict): 末端执行器目标姿态 {x, y, z, w}（必需）
- `is_straight_constraint` (bool): 是否启用直线约束，保持当前旋转姿态不变（可选，默认false）

**约束功能说明:**
当 `is_straight_constraint` 设置为 `true` 时，系统会：
1. 获取当前末端执行器的方向
2. 创建方向约束（OrientationConstraint），限制末端执行器在运动过程中保持当前的旋转姿态
3. 约束参数：
   - 容差：xyz轴各0.1弧度
   - 权重：1.0
   - 坐标系：base_link

**响应格式:**
```json
{ "message": "Success" }
```

**错误响应:**
```json
{ "error": "No JSON data provided" }
```
或
```json
{ "error": "Missing group_name" }
```
或
```json
{ "message": "Failed to reach goal position" }
```
- `400`: 请求数据错误（缺少必需字段或无JSON数据）
- `500`: 规划执行失败

### 4. 根据像素计算目标位姿

**接口:** `POST /calculate_target_position_from_pixel`

**描述:** 根据像素坐标(u,v)、深度d与绕X轴旋转角度theta，计算机器人基座坐标系下的目标位姿（position + orientation）。

**请求格式:**
```json
{
  "x": 320,
  "y": 240,
  "d": 0.5,
  "theta": -90,
  "group_name": "Right_arm"
}
```

**请求参数:**
- `x` (float): 像素横坐标 u（必需）
- `y` (float): 像素纵坐标 v（必需）
- `d` (float): 深度，单位米（必需）
- `theta` (float): 绕X轴旋转角度，单位度（可选，默认 0.0）
- `group_name` (str): 动作组名称（可选，默认 `"Right_arm"`）

**注意:** V2版本中，`camera_frame` 固定为 `"camera_color_link"`，`base_frame` 固定为 `"BASE_S"`，不再作为请求参数。

**成功响应:**
```json
{
  "success": true,
  "group_name": "Right_arm",
  "pose": {
    "position": { "x": 0.123, "y": 0.456, "z": 0.789 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  }
}
```

**错误响应:**
```json
{
  "success": false,
  "error": "Failed to calculate target pose from pixel"
}
```
- `400`: 无JSON数据或缺少必需字段 `x, y, d`
- `500`: 计算失败

**说明:**
- 内部调用 `caculate_target_pose_from_pixel_pose`，自动完成像素→相机→基座坐标转换及姿态计算。
- 相机坐标系固定为 `camera_color_link`，基座坐标系固定为 `BASE_S`。

### 5. 腿部电机动作

**接口:** `POST /leg_move`

**描述:** 发送腿部电机目标位置数组，通过 Action 客户端调用服务端执行。

**请求格式:**
```json
{
  "target_positions": [2500, 8500, 7500]
}
```

**请求参数:**
- `target_positions` (int[]): 目标位置数组（长度应与腿部电机数量一致，单位取决于服务端约定）

**成功响应:**
```json
{
  "success": true,
  "message": "Leg move command sent successfully",
  "target_positions": [2500, 8500, 7500]
}
```

**错误响应:**
```json
{
  "success": false,
  "error": "failed"
}
```
- `400`: 无JSON数据或缺少 `target_positions`
- `500`: 执行失败或超时

**说明:**
- 客户端会同步等待动作结果，默认超时 `10s`。服务端需要正确设置 `SUCCEEDED/ABORTED` 状态与结果消息。
