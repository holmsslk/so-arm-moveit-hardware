# SO-ARM MoveIt Workspace

一个基于 ROS2 Humble 的 6 自由度机械臂控制系统，支持真实硬件控制和 MoveIt 运动规划。

## 🎯 项目简介

SO-ARM 是一个完整的机械臂控制解决方案，集成了：

- **硬件接口**: 支持 Feetech ST-3215-C001 舵机的 ROS2 Control 硬件接口
- **运动规划**: 基于 MoveIt 2 的轨迹规划和执行
- **可视化**: RViz2 交互式控制界面

## 📦 包结构

```
so-arm_moveit_ws/
├── so_arm_description/      # 机械臂 URDF 模型和网格文件
├── so_arm_moveit_config/    # MoveIt 配置文件
├── so_arm_bringup/          # 启动文件和配置
├── so_arm_hardware/         # 硬件接口实现
├── so_arm_interfaces/       # 自定义消息和服务
├── so_arm_commander_cpp/    # 高级控制接口
└── so_arm_tasks/            # 任务执行模块
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

# 安装其他依赖
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-joint-state-publisher-gui
```

### 3. 编译项目

```bash
cd ~/so-arm_moveit_ws
colcon build
source install/setup.bash
```

### 4. 硬件设置

```bash
# 添加串口权限
sudo usermod -a -G dialout $USER
# 注销后重新登录

# 或临时授权
sudo chmod 666 /dev/ttyACM0
```

### 5. 启动系统

**完整系统（推荐）**:
```bash
ros2 launch so_arm_bringup so_arm_moveit_hardware.launch.py
```

**仅硬件测试**:
```bash
ros2 launch so_arm_bringup so_arm_hardware.launch.py
```

## 📚 文档

- **[PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md)** - 完整项目架构文档
- **[MOVEIT_HARDWARE_GUIDE.md](MOVEIT_HARDWARE_GUIDE.md)** - MoveIt 使用指南
- **[CLEANUP_SUMMARY.md](CLEANUP_SUMMARY.md)** - 项目清理总结
- **[src/so_arm_hardware/README.md](src/so_arm_hardware/README.md)** - 硬件接口详细文档

## 🔧 硬件规格

- **机械臂**: SO-ARM101 6-DOF
- **舵机**: Feetech ST-3215-C001 (6个)
- **通信**: UART @ 1,000,000 baud
- **接口**: USB to TTL (/dev/ttyACM0)
- **电源**: 5V

## 🎮 使用方法

### 交互式控制

1. 启动完整系统
2. 在 RViz 中使用 MotionPlanning 插件
3. 拖动末端交互式标记到目标位置
4. 点击 "Plan" 规划路径
5. 点击 "Execute" 执行运动

### 诊断工具

```bash
# 测试单个舵机
./fix_servo /dev/ttyACM0 1

# 查看控制器状态
ros2 control list_controllers

# 查看关节状态
ros2 topic echo /joint_states
```

## 🛠️ 开发

### 添加新功能

1. 在 `so_arm_commander_cpp` 中实现高级控制逻辑
2. 使用 `so_arm_moveit_hardware.launch.py` 测试
3. 更新文档到 `PROJECT_ARCHITECTURE.md`

### 调试

```bash
# 启用调试日志
ros2 launch so_arm_bringup so_arm_moveit_hardware.launch.py --ros-args --log-level debug

# 查看硬件组件
ros2 control list_hardware_components
```

## 📊 性能指标

| 指标 | 值 |
|-----|---|
| 控制频率 | 100 Hz |
| 舵机分辨率 | 12-bit (0-4095) |
| 最大速度 | 10 rad/s |
| 运动规划器 | OMPL (RRTConnect) |
| 启动时间 | ~10 秒 |

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

Apache-2.0

## 🙏 致谢

- ROS2 Control 团队
- MoveIt 2 团队
- Feetech 舵机驱动库

## 📧 联系方式

如有问题，请创建 Issue 或联系维护者。

---

**注意**: 使用前请确保机械臂周围无障碍物，注意安全！
