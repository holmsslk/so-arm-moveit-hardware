# SO-ARM MoveIt Workspace

一个基于 ROS2 Humble 的 6 自由度机械臂完整控制系统，支持真实硬件控制、MoveIt 运动规划和高级任务执行。

## 🎯 项目简介

SO-ARM 是一个完整的机械臂控制解决方案，提供从底层硬件接口到高级任务执行的全栈功能：

- **硬件接口**: 支持 Feetech ST-3215-C001 舵机的 ROS2 Control 硬件接口
- **运动规划**: 基于 MoveIt 2 的轨迹规划和执行
- **高级控制**: 简化的 C++ API，支持关节运动、笛卡尔运动、命名姿态
- **任务管理**: 任务序列编排和执行
- **可视化**: RViz2 交互式控制界面

## 📦 包结构

```
so-arm_moveit_ws/
├── so_arm_description/      # 机械臂 URDF 模型和网格文件
├── so_arm_moveit_config/    # MoveIt 配置（运动学、规划器、关节限制）
├── so_arm_bringup/          # 启动文件和配置
├── so_arm_hardware/         # ROS2 Control 硬件接口实现
├── so_arm_interfaces/       # 自定义消息和服务定义
├── so_arm_commander_cpp/    # 高级控制接口（简化 API）
└── so_arm_tasks/            # 任务管理和执行模块
```

## 🚀 快速开始

### 1. 系统要求

- Ubuntu 22.04
- ROS2 Humble
- MoveIt 2
- Feetech STS 舵机驱动库

### 2. 安装依赖

```bash
# 安装 ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装 MoveIt 2
sudo apt install ros-humble-moveit

# 安装 ROS2 Control 相关包
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-publisher-gui

# 安装其他工具
sudo apt install python3-colcon-common-extensions
```

### 3. 编译项目

```bash
cd ~/so-arm_moveit_ws
colcon build
source install/setup.bash
```

### 4. 硬件设置

```bash
# 添加串口权限（永久）
sudo usermod -a -G dialout $USER
# 注销后重新登录生效

# 或临时授权（每次重启后需要重新执行）
sudo chmod 666 /dev/ttyACM0
```

### 5. 启动系统

**方式一：完整系统（推荐）**
```bash
# 启动硬件接口 + MoveIt + RViz
ros2 launch so_arm_bringup so_arm_moveit_hardware.launch.py
```

**方式二：仅硬件测试**
```bash
# 仅启动硬件接口，用于测试舵机通信
ros2 launch so_arm_bringup so_arm_hardware.launch.py
```

## 🎮 使用方法

### 交互式控制（RViz）

1. 启动完整系统
2. 在 RViz 中使用 MotionPlanning 插件
3. 拖动末端执行器的交互式标记到目标位置
4. 点击 "Plan" 按钮规划运动路径
5. 检查路径是否合理，然后点击 "Execute" 执行

### 编程控制（C++ API）

使用 `so_arm_commander_cpp` 提供的高级 API：

```cpp
#include "so_arm_commander_cpp/so_arm_commander.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("my_robot_control");

  SoArmCommander arm(node);

  // 移动到命名姿态
  arm.moveToNamedPose("home");

  // 关节空间运动
  std::vector<double> joint_positions = {0.0, -0.5, 0.5, 0.0, 0.0, 0.0};
  arm.moveToJointPosition(joint_positions);

  // 笛卡尔空间运动
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.2;
  arm.moveToPose(target_pose);

  rclcpp::shutdown();
  return 0;
}
```

更多示例请参考 [so_arm_commander_cpp/examples](src/so_arm_commander_cpp/examples/)

### 任务序列执行

使用 `so_arm_tasks` 包执行复杂的任务序列：

```bash
# 运行任务示例
ros2 run so_arm_tasks task_examples
```

## 📚 文档

### 用户文档
- **[MOVEIT_HARDWARE_GUIDE.md](MOVEIT_HARDWARE_GUIDE.md)** - MoveIt 使用指南
- **[QUICK_START_NEXT.md](QUICK_START_NEXT.md)** - 快速开始指南

### 开发文档
- **[SO_ARM_COMMANDER_DESIGN.md](SO_ARM_COMMANDER_DESIGN.md)** - Commander 设计文档
- **[src/so_arm_commander_cpp/README.md](src/so_arm_commander_cpp/README.md)** - 高级控制 API 使用指南
- **[src/so_arm_hardware/README.md](src/so_arm_hardware/README.md)** - 硬件接口详细文档

## 🔧 硬件规格

| 项目 | 规格 |
|-----|------|
| **机械臂型号** | SO-ARM101 6-DOF |
| **舵机型号** | Feetech ST-3215-C001 (6个) |
| **通信协议** | UART @ 1,000,000 baud |
| **接口** | USB to TTL (/dev/ttyACM0) |
| **供电** | 5V |
| **负载** | ~500g |

## 📊 性能指标

| 指标 | 值 |
|-----|---|
| 控制频率 | 100 Hz |
| 舵机分辨率 | 12-bit (0-4095) |
| 位置精度 | ±0.5° |
| 最大速度 | 10 rad/s |
| 运动规划器 | OMPL (RRTConnect) |
| 启动时间 | ~10 秒 |

## 🔍 诊断工具

```bash
# 查看控制器状态
ros2 control list_controllers

# 查看硬件组件
ros2 control list_hardware_components

# 查看关节状态
ros2 topic echo /joint_states

# 查看规划场景
ros2 topic echo /planning_scene

# 查看执行轨迹
ros2 topic echo /execute_trajectory/status
```

## 🛠️ 开发指南

### 添加新的命名姿态

编辑 `src/so_arm_commander_cpp/config/named_poses.yaml`:

```yaml
named_poses:
  my_custom_pose:
    joint1: 0.0
    joint2: -1.57
    joint3: 1.57
    joint4: 0.0
    joint5: 0.0
    joint6: 0.0
```

### 调试模式

```bash
# 启用详细日志
ros2 launch so_arm_bringup so_arm_moveit_hardware.launch.py \
  --ros-args --log-level debug

# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看 URDF
check_urdf install/so_arm_description/share/so_arm_description/urdf/so101.urdf
```

### 自定义开发

1. **添加新控制逻辑**: 扩展 `so_arm_commander_cpp/src/so_arm_commander.cpp`
2. **添加新任务**: 在 `so_arm_tasks/src/` 创建新的任务类
3. **修改运动学参数**: 编辑 `so_arm_moveit_config/config/kinematics.yaml`
4. **调整规划器**: 编辑 `so_arm_moveit_config/config/ompl_planning.yaml`

## ⚠️ 常见问题

### 问题 1: 无法连接到硬件

```bash
# 检查设备是否存在
ls -l /dev/ttyACM*

# 检查权限
groups $USER  # 应该包含 dialout

# 检查是否有其他程序占用
sudo lsof /dev/ttyACM0
```

### 问题 2: 规划失败

- 检查目标位置是否在工作空间内
- 增加规划时间限制
- 尝试不同的起始姿态
- 查看碰撞检测设置

### 问题 3: 运动不平滑

- 调整速度和加速度限制（`joint_limits.yaml`）
- 增加轨迹点数量
- 检查控制器参数（`ros2_controllers.yaml`）

## 🤝 贡献

欢迎贡献代码、报告问题或提出改进建议！

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 打开 Pull Request

## 📄 许可证

Apache-2.0

## 🙏 致谢

- [ROS2 Control](https://github.com/ros-controls/ros2_control) - 硬件抽象层
- [MoveIt 2](https://moveit.ros.org/) - 运动规划框架
- Feetech - 舵机驱动库

## 📧 联系方式

- **GitHub Issues**: [提交问题](https://github.com/holmsslk/so-arm-moveit-hardware/issues)
- **项目主页**: https://github.com/holmsslk/so-arm-moveit-hardware

---

**⚠️ 安全提示**: 使用前确保机械臂周围无障碍物，首次运行建议低速测试，保持急停装置随时可用。
