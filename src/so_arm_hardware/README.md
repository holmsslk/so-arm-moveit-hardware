# SO-ARM Hardware Interface

ROS2 Control hardware interface for the SO-ARM101 robotic arm with Feetech STS3215 servos.

## Overview

This package provides a hardware interface plugin that enables ROS2 Control to communicate with real Feetech STS3215 servos via serial communication. It integrates the FTServo driver library to provide:

- Position control for 6-DOF joints
- Real-time state feedback (position, velocity)
- Servo health monitoring (temperature, current, voltage)
- Synchronous multi-servo control for improved performance

## Hardware Requirements

- **SO-ARM101 Robotic Arm**
- **6x Feetech STS3215 Servos** (IDs 1-6)
- **USB to TTL Serial Adapter** (typically appears as `/dev/ttyACM0` or `/dev/ttyUSB0`)
- **Communication**: UART @ 1,000,000 baud

## Installation

### 1. Set Up Serial Port Permissions

```bash
# Add your user to the dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in for changes to take effect

# Alternatively, temporarily grant permissions (for testing)
sudo chmod 666 /dev/ttyACM0
```

### 2. Build the Package

```bash
cd ~/so-arm_moveit_ws
colcon build --packages-select so_arm_hardware so_arm_description
source install/setup.bash
```

## Usage

### Launch with Real Hardware

**Option 1: Hardware Only (Basic Testing)**
```bash
ros2 launch so_arm_bringup so_arm_hardware.launch.py
```

**Option 2: Hardware + MoveIt + RViz (Recommended)**
```bash
ros2 launch so_arm_bringup so_arm_moveit_hardware.launch.py
```

This will:
1. Start the robot state publisher
2. Initialize the hardware interface (connect to servos)
3. Spawn controllers (joint_state_broadcaster, arm_controller)
4. Launch MoveIt move_group (motion planning)
5. Open RViz for visualization and interactive control

### Verify Hardware Connection

Check that servos are detected:

```bash
# Terminal 1: Launch the system
ros2 launch so_arm_bringup so_arm_hardware.launch.py

# Terminal 2: Check joint states
ros2 topic echo /joint_states

# Terminal 3: List controllers
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
arm_controller          [joint_trajectory_controller/JointTrajectoryController] active
```

### Test Servo Communication (Before ROS2 Launch)

Use the FTServo test tools:

```bash
cd ~/FTServo_Linux/examples/STSServo
./demo_scan /dev/ttyACM0
```

This should detect all 6 servos (IDs 1-6).

## Configuration

### Hardware Parameters

Edit `config/hardware_params.yaml` to customize:

- **Serial port**: `/dev/ttyACM0` (or `/dev/ttyUSB0`)
- **Baud rate**: `1000000`
- **Servo ID mapping**: Joint name → Servo ID
- **Position offsets**: Calibration offsets for each joint
- **Safety limits**: Max temperature (70°C), max current (1000mA)

### Servo ID Mapping

| Joint Name   | Servo ID | Function      |
|--------------|----------|---------------|
| Rotation     | 1        | Base rotation |
| Pitch        | 2        | Shoulder      |
| Elbow        | 3        | Elbow         |
| Wrist_Pitch  | 4        | Wrist pitch   |
| Wrist_Roll   | 5        | Wrist roll    |
| Jaw          | 6        | Gripper       |

### Position Conversion

- **Servo range**: 0 - 4095 (12-bit)
- **Center position**: 2048
- **Conversion formula**:
  ```
  radians = (servo_position - offset) * scale
  scale = 2π / 4096 ≈ 0.00153398 rad/unit
  ```

## Available Launch Files

The system provides two launch configurations:

### 1. Hardware Only
```bash
ros2 launch so_arm_bringup so_arm_hardware.launch.py
```
- Uses `so_arm_hardware/SoArmHardwareInterface`
- Starts ROS2 Control with hardware interface
- Includes RViz for basic visualization
- **Use case**: Testing hardware connection and basic control

### 2. Hardware + MoveIt (Recommended)
```bash
ros2 launch so_arm_bringup so_arm_moveit_hardware.launch.py
```
- Includes all features from hardware-only mode
- Adds MoveIt motion planning capabilities
- Interactive control via RViz MotionPlanning plugin
- **Use case**: Production use with motion planning

## Architecture

```
ROS2 Control
    ↓
SoArmHardwareInterface (C++)
    ↓
STSServo Driver (FTServo_Linux)
    ↓
Serial Communication (UART 1M baud)
    ↓
Feetech STS3215 Servos (6x)
```

### Key Classes

- **SoArmHardwareInterface**: Main hardware interface implementing `hardware_interface::SystemInterface`
- **STSServo**: High-level servo driver (from FTServo_Linux)
- **SMS_STS**: Protocol layer for Feetech communication

## Safety Features

The hardware interface includes:

- **Temperature monitoring**: Shuts down if servo exceeds 70°C
- **Current monitoring**: Warns if current exceeds 1000mA
- **Communication timeout**: Detects and reports communication failures
- **Servo health checks**: Validates all servos on startup

## Troubleshooting

### Serial Port Not Found

```bash
# Check USB devices
ls -l /dev/ttyACM* /dev/ttyUSB*

# Check USB connection
lsusb

# Check dmesg for device detection
dmesg | grep tty
```

### Permission Denied

```bash
sudo chmod 666 /dev/ttyACM0
# or
sudo usermod -a -G dialout $USER
# (then log out and back in)
```

### No Servos Detected

1. Check physical connections
2. Verify power supply to servos
3. Confirm baud rate is 1M:
   ```bash
   cd ~/FTServo_Linux/examples/SMS_STS
   ./ScanServoMultiBaud /dev/ttyACM0
   ```

### Controller Fails to Start

Check logs:
```bash
ros2 control list_hardware_components
ros2 control list_controllers
```

View detailed logs:
```bash
ros2 launch so_arm_bringup so_arm_hardware.launch.py --log-level debug
```

### Servo Overheating

If temperature exceeds 70°C:
- Reduce speed/acceleration parameters
- Allow cooling time
- Check for mechanical binding

## Development

### Running Tests

```bash
# Build with tests
colcon build --packages-select so_arm_hardware

# Run servo communication test
cd ~/FTServo_Linux/examples/STSServo
./demo_basic /dev/ttyACM0
```

### Debugging

Enable debug logging:
```bash
ros2 launch so_arm_bringup so_arm_hardware.launch.py --ros-args --log-level debug
```

## License

Apache-2.0

## References

- [FTServo_Linux Driver](~/FTServo_Linux)
- [ROS2 Control Documentation](https://control.ros.org)
- [Feetech STS3215 Datasheet](https://www.feetechrc.com)
