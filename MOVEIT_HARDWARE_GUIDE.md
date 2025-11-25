# 🤖 MoveIt + 真实硬件 完整控制指南

## 🎯 概述

现在你可以使用 **MoveIt 运动规划** 来控制真实的机械臂了！

这个系统包括：
- ✅ 硬件接口（ROS2 Control）
- ✅ MoveIt 运动规划
- ✅ RViz 可视化和交互式控制
- ✅ 碰撞检测和路径规划

---

## 🚀 快速启动

### 方法 1：一键启动完整系统（推荐）

```bash
cd ~/so-arm_moveit_ws
source install/setup.bash
ros2 launch so_arm_bringup so_arm_moveit_hardware.launch.py
```

**这会自动启动：**
1. ✅ 硬件接口（连接真实舵机）
2. ✅ 控制器（joint_state_broadcaster + arm_controller）
3. ✅ MoveIt move_group（运动规划节点）
4. ✅ RViz（MoveIt 可视化界面）

**等待时间：**
- 初始化硬件：~2秒
- 启动控制器：~3秒
- MoveIt 启动：~2秒
- RViz 启动：~3秒
- **总计约 10 秒完全就绪**

---

## 🎮 使用 MoveIt 控制机械臂

### 在 RViz 中交互式规划

启动完成后，RViz 会自动打开 MoveIt 界面：

#### 1. **MotionPlanning 面板**

左侧会有一个 `MotionPlanning` 面板，包含：

- **Planning Group**: 选择 `arm`
- **Query Goal State**: 设置目标位置

#### 2. **交互式控制 End-Effector**

- 在 RViz 3D 视图中，会看到机械臂末端的**交互式标记**（Interactive Marker）
- 用鼠标拖动这个标记到目标位置
- 点击 `Plan` 按钮生成路径
- 点击 `Execute` 按钮执行运动

#### 3. **设置目标状态的方式**

**方式 A - 拖动交互式标记**：
1. 在 3D 视图中找到末端的球形/箭头标记
2. 拖动到目标位置
3. 点击 `Plan`
4. 查看规划的路径（橙色轨迹）
5. 确认后点击 `Execute`

**方式 B - 选择预定义姿态**：
1. 在 `MotionPlanning` 面板
2. `Select Goal State` → 选择 `<current>` 或 `home`
3. 或者手动设置每个关节角度
4. 点击 `Update`
5. 点击 `Plan`
6. 点击 `Execute`

**方式 C - 随机目标**：
1. 点击 `Random Valid` 按钮
2. 自动生成一个有效的随机目标
3. 点击 `Plan`
4. 点击 `Execute`

---

## 📊 MoveIt 界面详解

### MotionPlanning 面板的主要部分

#### **Planning** 标签
- `Planning Group`: 选择 `arm`（包含所有6个关节）
- `Query Start State`: 设置起始状态（通常是 `<current>`）
- `Query Goal State`: 设置目标状态
- `Plan` 按钮: 计算运动路径
- `Execute` 按钮: 执行规划的路径
- `Plan & Execute` 按钮: 一键规划并执行

#### **Scene Objects** 标签
- 添加障碍物（用于碰撞检测）
- 导入/导出场景
- 管理场景对象

#### **Stored States** 标签
- 保存常用的机械臂姿态
- 默认有 `home` 状态（所有关节为0）

---

## 🎯 基本操作流程

### 流程 1：移动到 Home 位置

```
1. MotionPlanning 面板
2. Query Goal State → Select → home
3. 点击 Plan 按钮
4. 查看规划路径（橙色）
5. 点击 Execute 按钮
```

**真实机械臂会平滑地移动到 home 位置！**

### 流程 2：交互式拖动控制

```
1. 在 3D 视图中找到末端的交互式标记
2. 用鼠标拖动标记到新位置
3. 旋转标记调整方向（如果需要）
4. 点击 Plan
5. 确认路径后点击 Execute
```

### 流程 3：手动设置关节角度

```
1. MotionPlanning 面板
2. Query Goal State → Joints 标签
3. 手动调整每个关节的滑块：
   - Rotation: -π 到 π
   - Pitch: -π/2 到 π/2
   - Elbow: -π 到 π
   - Wrist_Pitch: -π/2 到 π/2
   - Wrist_Roll: -π 到 π
   - Jaw: -π/4 到 π/4
4. 点击 Update
5. 点击 Plan
6. 点击 Execute
```

---

## 🔧 高级功能

### 1. 规划算法选择

在 `MotionPlanning` 面板:
- `Planner` 下拉菜单
- 可选：RRTConnect, RRT, PRM, EST 等
- **推荐**: RRTConnect（速度快，成功率高）

### 2. 规划参数调整

- `Planning Time (s)`: 规划时间限制（默认5秒）
- `Planning Attempts`: 尝试次数（默认10次）
- `Max Velocity Scaling`: 速度缩放（0.0-1.0）
- `Max Acceleration Scaling`: 加速度缩放（0.0-1.0）

**首次测试建议**：
- Max Velocity Scaling: 0.3（30%速度）
- Max Acceleration Scaling: 0.3（30%加速度）

### 3. 添加障碍物

在 `Scene Objects` 标签:
```
1. 点击 Add Obstacle
2. 选择形状（Box, Sphere, Cylinder）
3. 设置尺寸和位置
4. 点击 Add
```

MoveIt 会自动避开障碍物进行规划！

### 4. 轨迹可视化

- **橙色路径**: 规划的运动轨迹
- **绿色机械臂**: 目标状态
- **灰色机械臂**: 当前状态

---

## 🎨 RViz 显示设置

### 推荐的显示配置

RViz 会自动加载 MoveIt 配置，包括：

1. **MotionPlanning Display**
   - Planned Path: 显示规划的轨迹
   - Scene Robot: 显示机械臂模型
   - Planning Scene: 显示场景和障碍物

2. **TF Display**
   - 显示所有坐标系

3. **RobotModel**
   - 显示机械臂 URDF 模型

---

## 🔥 实际测试示例

### 示例 1：简单的往返运动

```
步骤 1: 移动到 home
  - Select Goal State → home
  - Plan & Execute

步骤 2: 移动到自定义位置
  - 拖动交互式标记到右侧
  - Plan & Execute

步骤 3: 返回 home
  - Select Goal State → home
  - Plan & Execute
```

### 示例 2：测试工作空间边界

```
1. 使用 Random Valid 生成随机目标
2. 重复点击几次，观察可达空间
3. 对每个目标点击 Plan & Execute
4. 观察机械臂的运动
```

### 示例 3：精确控制

```
1. Joints 标签手动设置：
   Rotation: 0.5 rad
   Pitch: 0.3 rad
   Elbow: 0.5 rad
   其他: 0.0
2. Update → Plan → Execute
3. 机械臂移动到精确的关节角度
```

---

## 📡 通过代码控制（Python）

虽然 RViz 很好用，但你也可以用 Python 编程控制：

### Python MoveIt 接口示例

创建文件 `test_moveit_control.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes
from moveit.planning import MoveItPy

class MoveItController(Node):
    def __init__(self):
        super().__init__('moveit_controller')
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("arm")

    def move_to_home(self):
        """移动到 home 位置"""
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home")

        # 规划
        plan_result = self.arm.plan()

        if plan_result:
            self.get_logger().info("规划成功，执行中...")
            # 执行
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
        else:
            self.get_logger().error("规划失败")

    def move_to_joint_values(self, joint_values):
        """移动到指定关节角度"""
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration=joint_values)

        plan_result = self.arm.plan()

        if plan_result:
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])

def main():
    rclpy.init()
    controller = MoveItController()

    # 移动到 home
    controller.move_to_home()

    # 等待
    rclpy.spin_once(controller, timeout_sec=5.0)

    # 移动到自定义位置
    joint_values = [0.5, 0.3, 0.5, 0.0, 0.0, 0.0]
    controller.move_to_joint_values(joint_values)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

运行：
```bash
chmod +x test_moveit_control.py
python3 test_moveit_control.py
```

---

## ⚠️ 安全注意事项

### 首次使用 MoveIt 控制真实硬件

1. **降低速度和加速度**
   - Max Velocity Scaling: 0.1-0.3
   - Max Acceleration Scaling: 0.1-0.3

2. **小心工作空间边界**
   - MoveIt 可能规划超出物理限制的路径
   - 首次测试使用小幅度运动

3. **观察规划路径**
   - 执行前仔细检查橙色轨迹
   - 确保路径合理且安全

4. **紧急停止**
   - 随时准备按 Ctrl+C 停止程序
   - 或断开电源

5. **检查关节限位**
   - 确保 `joint_limits.yaml` 配置正确
   - 防止机械臂运动到危险位置

---

## 🔧 故障排除

### 问题 1：MoveIt 规划失败

**症状**：点击 Plan 显示 "ABORTED" 或 "PLANNING_FAILED"

**可能原因**：
- 目标位置超出工作空间
- 目标位置与障碍物碰撞
- 规划时间不足

**解决**：
```
1. 增加 Planning Time 到 10 秒
2. 使用 Random Valid 生成有效目标
3. 检查关节限位配置
```

### 问题 2：Execute 后机械臂不动

**症状**：Execute 成功但机械臂无响应

**检查**：
```bash
# 检查控制器状态
ros2 control list_controllers

# 应该显示：
# arm_controller [...] active

# 检查硬件接口
ros2 control list_hardware_interfaces
```

### 问题 3：RViz 中看不到交互式标记

**解决**：
1. MotionPlanning 面板 → Planning Request 标签
2. 确保 `Query Goal State` 已选择
3. 重新选择 Planning Group

### 问题 4：规划的路径不平滑

**原因**：默认规划器可能产生抖动路径

**解决**：
```
1. 切换到不同的规划器：
   - RRTConnect (推荐)
   - RRT
   - PRM

2. 启用时间参数化：
   - Planning → Use Time Parameterization (已默认启用)
```

---

## 📊 性能监控

### 监控 MoveIt 状态

```bash
# MoveIt move_group 状态
ros2 topic echo /move_group/status

# 规划场景
ros2 topic echo /move_group/monitored_planning_scene

# 当前状态
ros2 topic echo /move_group/display_planned_path
```

### 监控硬件状态

```bash
# 关节状态（100Hz）
ros2 topic hz /joint_states

# 控制器状态
ros2 topic echo /arm_controller/state

# 舵机诊断
./fix_servo /dev/ttyACM0 1
```

---

## 🎯 推荐工作流程

### 日常使用流程

```
1. 启动系统：
   ros2 launch so_arm_bringup so_arm_moveit_hardware.launch.py

2. 等待 RViz 出现（约10秒）

3. 移动到 home 位置（确认系统正常）

4. 开始交互式控制或编程控制

5. 完成后 Ctrl+C 停止
```

### 开发调试流程

```
1. 终端1：启动 MoveIt 系统

2. 终端2：运行测试脚本
   python3 test_moveit_control.py

3. 观察 RViz 中的运动

4. 调整代码并重复
```

---

## 📚 相关文档

- **MoveIt 2 教程**: https://moveit.picknik.ai/main/index.html
- **ROS2 Control**: https://control.ros.org
- **本项目文档**:
  - [START_HERE.md](START_HERE.md) - 快速开始
  - [RVIZ_CONTROL_GUIDE.md](RVIZ_CONTROL_GUIDE.md) - RViz 基础
  - [TESTING_GUIDE.md](TESTING_GUIDE.md) - 完整测试指南

---

## 🚀 快速命令参考

```bash
# 启动完整 MoveIt 系统
ros2 launch so_arm_bringup so_arm_moveit_hardware.launch.py

# 仅启动硬件接口（不含 MoveIt）
ros2 launch so_arm_bringup so_arm_hardware_simple.launch.py

# 启动硬件 + RViz（不含 MoveIt）
ros2 launch so_arm_bringup so_arm_hardware_with_rviz.launch.py

# 检查 MoveIt 服务
ros2 service list | grep move_group

# 查看规划组
ros2 param get /move_group planning_scene_monitor/robot_description

# 查看当前状态
ros2 topic echo /joint_states --once
```

---

**现在开始用 MoveIt 控制你的机械臂吧！** 🤖✨

记住：首次使用降低速度，观察规划路径，确保安全！
