# 🏗️ SoArmCommander.hpp 设计文档

## 📚 目录
1. [设计理念](#设计理念)
2. [头文件结构](#头文件结构)
3. [逐步构建指南](#逐步构建指南)
4. [完整代码详解](#完整代码详解)
5. [使用示例](#使用示例)

---

## 🎯 设计理念

### 为什么需要 SoArmCommander？

**问题**: MoveIt 的原生 API 较复杂，简单的任务需要很多代码：

```cpp
// 使用原生 MoveIt API (复杂)
auto node = rclcpp::Node::make_shared("my_node");
auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node, "arm");
move_group->setJointValueTarget(positions);
moveit::planning_interface::MoveGroupInterface::Plan plan;
bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
if (success) {
    move_group->execute(plan);
}
```

**解决方案**: SoArmCommander 提供简化接口：

```cpp
// 使用 SoArmCommander (简单)
SoArmCommander commander;
commander.moveToJointPosition({0, 0, 0, 0, 0, 0});
```

### 设计目标

1. **易用性**: 一行代码完成常见操作
2. **安全性**: 内置边界检查和错误处理
3. **可扩展性**: 易于添加新功能
4. **高性能**: 优化的规划参数

---

## 📋 头文件结构

### 整体架构

```
so_arm_commander.hpp
├── 头部保护 (#ifndef/#define)
├── 包含文件
│   ├── ROS2 核心
│   ├── MoveIt 接口
│   ├── 消息类型
│   └── 工具库
├── 命名空间 (可选)
├── 类定义
│   ├── 构造函数/析构函数
│   ├── 公共接口
│   │   ├── 基础运动
│   │   ├── 笛卡尔运动
│   │   ├── 预定义姿态
│   │   ├── 夹爪控制
│   │   └── 高级功能
│   └── 私有成员
│       ├── MoveIt 接口指针
│       ├── 配置参数
│       ├── 状态变量
│       └── 辅助函数
└── 尾部保护 (#endif)
```

---

## 🔨 逐步构建指南

### Step 1: 头部保护和包含文件

**目的**: 防止重复包含，引入必要的依赖

```cpp
#ifndef SO_ARM_COMMANDER_CPP__SO_ARM_COMMANDER_HPP_
#define SO_ARM_COMMANDER_CPP__SO_ARM_COMMANDER_HPP_

// === ROS2 核心 ===
#include <rclcpp/rclcpp.hpp>

// === MoveIt 核心接口 ===
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// === 消息类型 ===
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// === 标准库 ===
#include <vector>
#include <string>
#include <map>
#include <memory>

// === TF2 (用于坐标转换) ===
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
```

**详解**:
- `rclcpp`: ROS2 的 C++ 客户端库
- `move_group_interface`: MoveIt 的主要控制接口
- `planning_scene_interface`: 用于添加障碍物（可选）
- `geometry_msgs`: 位姿消息类型
- `tf2_geometry_msgs`: 四元数和 RPY 转换

---

### Step 2: 类声明基础框架

```cpp
namespace so_arm_commander
{

class SoArmCommander : public rclcpp::Node
{
public:
    // 构造函数
    explicit SoArmCommander(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // 析构函数
    ~SoArmCommander();

private:
    // 私有成员将在后面定义
};

}  // namespace so_arm_commander
```

**关键点**:
- 继承 `rclcpp::Node`: 使其成为 ROS2 节点
- 使用命名空间避免命名冲突
- `explicit` 防止隐式转换

---

### Step 3: 定义公共接口

#### 3.1 基础运动函数

```cpp
public:
    /**
     * @brief 移动到零位（Home位置）
     * @return 成功返回 true
     */
    bool goHome();

    /**
     * @brief 移动到指定关节位置
     * @param positions 6个关节角度（弧度）
     * @return 成功返回 true
     */
    bool moveToJointPosition(const std::vector<double>& positions);

    /**
     * @brief 获取当前关节位置
     * @return 当前关节角度向量
     */
    std::vector<double> getCurrentJointPositions();
```

**设计考虑**:
- 返回 `bool` 表示成功/失败
- 使用 `const std::vector<double>&` 避免拷贝
- 添加 Doxygen 注释

---

#### 3.2 笛卡尔空间运动

```cpp
    /**
     * @brief 移动到指定笛卡尔位姿
     * @param target 目标位姿（Pose 消息）
     * @return 成功返回 true
     */
    bool moveToPose(const geometry_msgs::msg::Pose& target);

    /**
     * @brief 移动到指定笛卡尔坐标（便捷函数）
     * @param x, y, z 位置（米）
     * @param roll, pitch, yaw 姿态（弧度）
     * @return 成功返回 true
     */
    bool moveToPose(double x, double y, double z,
                    double roll, double pitch, double yaw);

    /**
     * @brief 笛卡尔直线运动
     * @param target 目标位姿
     * @param velocity_scaling 速度缩放因子 (0.0-1.0)
     * @return 成功返回 true
     */
    bool moveLinear(const geometry_msgs::msg::Pose& target,
                    double velocity_scaling = 0.5);

    /**
     * @brief 获取当前末端位姿
     * @return 当前位姿
     */
    geometry_msgs::msg::Pose getCurrentPose();
```

**函数重载**:
- 提供多种便捷形式
- `moveToPose(Pose)`: 直接使用 Pose 消息
- `moveToPose(x,y,z,r,p,y)`: 更直观的参数

---

#### 3.3 预定义姿态

```cpp
    /**
     * @brief 移动到命名姿态
     * @param target_name 姿态名称 ("home", "ready", "observe"等)
     * @return 成功返回 true
     */
    bool setNamedTarget(const std::string& target_name);

    /**
     * @brief 添加新的命名姿态
     * @param name 姿态名称
     * @param positions 关节位置
     * @return 成功返回 true
     */
    bool addNamedPose(const std::string& name,
                      const std::vector<double>& positions);

    /**
     * @brief 获取所有可用的命名姿态
     * @return 姿态名称列表
     */
    std::vector<std::string> getNamedPosesList();
```

**预定义姿态**:
- 存储常用位置
- 便于快速切换
- 可动态添加

---

#### 3.4 夹爪控制

```cpp
    /**
     * @brief 打开夹爪
     * @return 成功返回 true
     */
    bool openGripper();

    /**
     * @brief 关闭夹爪
     * @return 成功返回 true
     */
    bool closeGripper();

    /**
     * @brief 设置夹爪位置
     * @param position 夹爪开度 (0.0=闭合, 1.0=全开)
     * @return 成功返回 true
     */
    bool setGripperPosition(double position);

    /**
     * @brief 获取当前夹爪状态
     * @return 夹爪开度 (0.0-1.0)
     */
    double getGripperPosition();
```

**设计**:
- 归一化控制 (0.0-1.0)
- 与实际舵机角度解耦
- 易于理解和使用

---

#### 3.5 高级功能

```cpp
    /**
     * @brief 抓取物体
     * @param target 抓取位置
     * @param approach_distance 接近距离（米）
     * @return 成功返回 true
     */
    bool pick(const geometry_msgs::msg::Pose& target,
              double approach_distance = 0.10);

    /**
     * @brief 放置物体
     * @param target 放置位置
     * @param retreat_distance 退出距离（米）
     * @return 成功返回 true
     */
    bool place(const geometry_msgs::msg::Pose& target,
               double retreat_distance = 0.10);

    /**
     * @brief 执行 Pick & Place 序列
     * @param pick_pose 抓取位置
     * @param place_pose 放置位置
     * @return 成功返回 true
     */
    bool pickAndPlace(const geometry_msgs::msg::Pose& pick_pose,
                      const geometry_msgs::msg::Pose& place_pose);
```

**高级功能**:
- 封装复杂操作
- 自动处理中间步骤
- 提供合理默认参数

---

### Step 4: 配置和查询函数

```cpp
    /**
     * @brief 设置规划时间限制
     * @param seconds 秒数
     */
    void setPlanningTime(double seconds);

    /**
     * @brief 设置速度缩放因子
     * @param factor 缩放因子 (0.0-1.0)
     */
    void setVelocityScaling(double factor);

    /**
     * @brief 设置加速度缩放因子
     * @param factor 缩放因子 (0.0-1.0)
     */
    void setAccelerationScaling(double factor);

    /**
     * @brief 设置工作空间边界
     * @param min_x, max_x, min_y, max_y, min_z, max_z 边界值（米）
     */
    void setWorkspaceBounds(double min_x, double max_x,
                            double min_y, double max_y,
                            double min_z, double max_z);

    /**
     * @brief 检查位置是否在安全工作空间内
     * @param x, y, z 位置坐标
     * @return 在工作空间内返回 true
     */
    bool isInWorkspace(double x, double y, double z);
```

---

### Step 5: 私有成员变量

```cpp
private:
    // === MoveIt 接口 ===
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

    // === 配置参数 ===
    std::string planning_group_;           // 规划组名称 (默认 "arm")
    double planning_time_;                 // 规划时间限制
    double velocity_scaling_factor_;       // 速度缩放
    double acceleration_scaling_factor_;   // 加速度缩放

    // === 预定义姿态 ===
    std::map<std::string, std::vector<double>> named_poses_;

    // === 夹爪参数 ===
    double gripper_open_position_;         // 夹爪全开位置
    double gripper_closed_position_;       // 夹爪闭合位置
    size_t gripper_joint_index_;           // 夹爪关节索引（通常是5）

    // === 工作空间限制 ===
    double workspace_min_x_, workspace_max_x_;
    double workspace_min_y_, workspace_max_y_;
    double workspace_min_z_, workspace_max_z_;
```

**设计要点**:
- 使用 `std::shared_ptr` 管理 MoveIt 对象
- 分类组织成员变量
- 添加注释说明用途

---

### Step 6: 私有辅助函数

```cpp
private:
    /**
     * @brief 初始化 MoveIt 接口
     * @return 成功返回 true
     */
    bool initializeMoveIt();

    /**
     * @brief 加载预定义姿态
     * @return 成功返回 true
     */
    bool loadNamedPoses();

    /**
     * @brief 验证关节位置是否有效
     * @param positions 关节位置
     * @return 有效返回 true
     */
    bool validateJointPositions(const std::vector<double>& positions);

    /**
     * @brief 将 RPY 转换为 Pose
     * @param x, y, z 位置
     * @param roll, pitch, yaw 姿态
     * @return Pose 消息
     */
    geometry_msgs::msg::Pose createPose(double x, double y, double z,
                                         double roll, double pitch, double yaw);

    /**
     * @brief 执行规划和运动
     * @return 成功返回 true
     */
    bool planAndExecute();

    /**
     * @brief 等待运动完成
     * @param timeout 超时时间（秒）
     * @return 成功返回 true
     */
    bool waitForExecution(double timeout = 10.0);
```

---

## 📝 完整代码详解

现在让我们把所有部分组合成完整的头文件：

```cpp
#ifndef SO_ARM_COMMANDER_CPP__SO_ARM_COMMANDER_HPP_
#define SO_ARM_COMMANDER_CPP__SO_ARM_COMMANDER_HPP_

// === 包含文件 ===
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <string>
#include <map>
#include <memory>

namespace so_arm_commander
{

/**
 * @class SoArmCommander
 * @brief 高级机械臂控制接口，简化 MoveIt 操作
 *
 * 提供易用的 API 来控制 SO-ARM 机械臂，包括：
 * - 基础运动控制
 * - 笛卡尔空间运动
 * - 预定义姿态
 * - 夹爪控制
 * - Pick & Place 操作
 *
 * @example
 * @code
 * auto commander = std::make_shared<SoArmCommander>();
 * commander->goHome();
 * commander->moveToPose(0.2, 0.1, 0.3, 0, M_PI/4, 0);
 * @endcode
 */
class SoArmCommander : public rclcpp::Node
{
public:
    // ========================================
    // 构造函数和析构函数
    // ========================================

    /**
     * @brief 构造函数
     * @param options ROS2 节点选项
     */
    explicit SoArmCommander(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~SoArmCommander();

    // ========================================
    // 基础运动控制
    // ========================================

    /**
     * @brief 移动到零位（Home位置）
     * @return 成功返回 true
     */
    bool goHome();

    /**
     * @brief 移动到指定关节位置
     * @param positions 6个关节角度（弧度）
     * @return 成功返回 true
     */
    bool moveToJointPosition(const std::vector<double>& positions);

    /**
     * @brief 获取当前关节位置
     * @return 当前关节角度向量
     */
    std::vector<double> getCurrentJointPositions();

    // ========================================
    // 笛卡尔空间运动
    // ========================================

    /**
     * @brief 移动到指定笛卡尔位姿
     * @param target 目标位姿
     * @return 成功返回 true
     */
    bool moveToPose(const geometry_msgs::msg::Pose& target);

    /**
     * @brief 移动到指定笛卡尔坐标（便捷函数）
     * @param x, y, z 位置（米）
     * @param roll, pitch, yaw 姿态（弧度）
     * @return 成功返回 true
     */
    bool moveToPose(double x, double y, double z,
                    double roll, double pitch, double yaw);

    /**
     * @brief 笛卡尔直线运动
     * @param target 目标位姿
     * @param velocity_scaling 速度缩放因子 (0.0-1.0)
     * @return 成功返回 true
     */
    bool moveLinear(const geometry_msgs::msg::Pose& target,
                    double velocity_scaling = 0.5);

    /**
     * @brief 获取当前末端位姿
     * @return 当前位姿
     */
    geometry_msgs::msg::Pose getCurrentPose();

    // ========================================
    // 预定义姿态
    // ========================================

    /**
     * @brief 移动到命名姿态
     * @param target_name 姿态名称
     * @return 成功返回 true
     */
    bool setNamedTarget(const std::string& target_name);

    /**
     * @brief 添加新的命名姿态
     * @param name 姿态名称
     * @param positions 关节位置
     * @return 成功返回 true
     */
    bool addNamedPose(const std::string& name,
                      const std::vector<double>& positions);

    /**
     * @brief 获取所有可用的命名姿态
     * @return 姿态名称列表
     */
    std::vector<std::string> getNamedPosesList();

    // ========================================
    // 夹爪控制
    // ========================================

    /**
     * @brief 打开夹爪
     * @return 成功返回 true
     */
    bool openGripper();

    /**
     * @brief 关闭夹爪
     * @return 成功返回 true
     */
    bool closeGripper();

    /**
     * @brief 设置夹爪位置
     * @param position 夹爪开度 (0.0=闭合, 1.0=全开)
     * @return 成功返回 true
     */
    bool setGripperPosition(double position);

    /**
     * @brief 获取当前夹爪状态
     * @return 夹爪开度 (0.0-1.0)
     */
    double getGripperPosition();

    // ========================================
    // 高级功能
    // ========================================

    /**
     * @brief 抓取物体
     * @param target 抓取位置
     * @param approach_distance 接近距离（米）
     * @return 成功返回 true
     */
    bool pick(const geometry_msgs::msg::Pose& target,
              double approach_distance = 0.10);

    /**
     * @brief 放置物体
     * @param target 放置位置
     * @param retreat_distance 退出距离（米）
     * @return 成功返回 true
     */
    bool place(const geometry_msgs::msg::Pose& target,
               double retreat_distance = 0.10);

    /**
     * @brief 执行 Pick & Place 序列
     * @param pick_pose 抓取位置
     * @param place_pose 放置位置
     * @return 成功返回 true
     */
    bool pickAndPlace(const geometry_msgs::msg::Pose& pick_pose,
                      const geometry_msgs::msg::Pose& place_pose);

    // ========================================
    // 配置函数
    // ========================================

    /**
     * @brief 设置规划时间限制
     * @param seconds 秒数
     */
    void setPlanningTime(double seconds);

    /**
     * @brief 设置速度缩放因子
     * @param factor 缩放因子 (0.0-1.0)
     */
    void setVelocityScaling(double factor);

    /**
     * @brief 设置加速度缩放因子
     * @param factor 缩放因子 (0.0-1.0)
     */
    void setAccelerationScaling(double factor);

    /**
     * @brief 设置工作空间边界
     * @param min_x, max_x, min_y, max_y, min_z, max_z 边界值（米）
     */
    void setWorkspaceBounds(double min_x, double max_x,
                            double min_y, double max_y,
                            double min_z, double max_z);

    /**
     * @brief 检查位置是否在安全工作空间内
     * @param x, y, z 位置坐标
     * @return 在工作空间内返回 true
     */
    bool isInWorkspace(double x, double y, double z);

private:
    // ========================================
    // MoveIt 接口
    // ========================================
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

    // ========================================
    // 配置参数
    // ========================================
    std::string planning_group_;           // "arm"
    double planning_time_;                 // 5.0 秒
    double velocity_scaling_factor_;       // 0.5
    double acceleration_scaling_factor_;   // 0.5

    // ========================================
    // 预定义姿态
    // ========================================
    std::map<std::string, std::vector<double>> named_poses_;

    // ========================================
    // 夹爪参数
    // ========================================
    double gripper_open_position_;         // 1.0 (弧度)
    double gripper_closed_position_;       // 0.0 (弧度)
    size_t gripper_joint_index_;           // 5 (Jaw)

    // ========================================
    // 工作空间限制
    // ========================================
    double workspace_min_x_, workspace_max_x_;
    double workspace_min_y_, workspace_max_y_;
    double workspace_min_z_, workspace_max_z_;

    // ========================================
    // 辅助函数
    // ========================================

    /**
     * @brief 初始化 MoveIt 接口
     */
    bool initializeMoveIt();

    /**
     * @brief 加载预定义姿态
     */
    bool loadNamedPoses();

    /**
     * @brief 验证关节位置
     */
    bool validateJointPositions(const std::vector<double>& positions);

    /**
     * @brief 创建 Pose 消息
     */
    geometry_msgs::msg::Pose createPose(double x, double y, double z,
                                         double roll, double pitch, double yaw);

    /**
     * @brief 执行规划和运动
     */
    bool planAndExecute();

    /**
     * @brief 等待运动完成
     */
    bool waitForExecution(double timeout = 10.0);
};

}  // namespace so_arm_commander

#endif  // SO_ARM_COMMANDER_CPP__SO_ARM_COMMANDER_HPP_
```

---

## 💡 使用示例

### 示例 1: 基础运动

```cpp
#include "so_arm_commander_cpp/so_arm_commander.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 创建 Commander
    auto commander = std::make_shared<so_arm_commander::SoArmCommander>();

    // 移动到零位
    commander->goHome();

    // 移动到指定关节位置
    std::vector<double> target = {0.5, 0.3, -0.5, 0.2, 0.0, 0.0};
    commander->moveToJointPosition(target);

    rclcpp::shutdown();
    return 0;
}
```

### 示例 2: 笛卡尔运动

```cpp
// 移动到指定位置
commander->moveToPose(0.2, 0.1, 0.3,  // x, y, z
                      0, M_PI/4, 0);   // roll, pitch, yaw

// 或使用 Pose 消息
geometry_msgs::msg::Pose target;
target.position.x = 0.2;
target.position.y = 0.1;
target.position.z = 0.3;
commander->moveToPose(target);
```

### 示例 3: Pick & Place

```cpp
geometry_msgs::msg::Pose pick_pose;
pick_pose.position.x = 0.25;
pick_pose.position.y = 0.0;
pick_pose.position.z = 0.1;

geometry_msgs::msg::Pose place_pose;
place_pose.position.x = 0.15;
place_pose.position.y = 0.2;
place_pose.position.z = 0.1;

// 执行抓取-放置
commander->pickAndPlace(pick_pose, place_pose);
```

---

## 🎯 下一步

头文件创建完成后：
1. 创建 `so_arm_commander.cpp` 实现文件
2. 实现每个函数
3. 测试基础功能

**准备好开始实现了吗？** 🚀
