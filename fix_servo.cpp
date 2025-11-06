/*
 * SO-ARM 舵机诊断和修复工具
 * 用于测试和解锁舵机
 */

#include <iostream>
#include <unistd.h>
#include "STSServo.h"

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "用法: " << argv[0] << " <串口设备> [舵机ID]" << std::endl;
        std::cout << "示例: " << argv[0] << " /dev/ttyACM0 3" << std::endl;
        return 1;
    }

    const char* port = argv[1];
    int target_id = (argc > 2) ? std::atoi(argv[2]) : 0;

    std::cout << "=========================================" << std::endl;
    std::cout << "SO-ARM 舵机诊断和修复工具" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << std::endl;

    // 1. 初始化
    STSServo servo;
    std::cout << "1️⃣  初始化串口: " << port << std::endl;
    if (!servo.init(port, 1000000)) {
        std::cerr << "❌ 初始化失败!" << std::endl;
        return 1;
    }
    std::cout << "✅ 串口初始化成功" << std::endl;
    std::cout << std::endl;

    // 2. 扫描舵机
    std::cout << "2️⃣  扫描舵机..." << std::endl;
    std::vector<u8> servos = servo.scanServos(1, 20);
    std::cout << "✅ 发现 " << servos.size() << " 个舵机: ";
    for (auto id : servos) {
        std::cout << (int)id << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;

    // 如果指定了舵机ID，进行详细测试
    if (target_id > 0) {
        std::cout << "3️⃣  诊断舵机 ID " << target_id << "..." << std::endl;

        // 检查舵机是否在线
        int ping_result = servo.ping(target_id);
        if (ping_result < 0) {
            std::cerr << "❌ 舵机 " << target_id << " 无响应!" << std::endl;
            return 1;
        }
        std::cout << "✅ 舵机 " << target_id << " 在线" << std::endl;

        // 读取舵机信息
        ServoInfo info;
        if (servo.readServoInfo(target_id, info)) {
            std::cout << std::endl;
            std::cout << "📊 舵机状态:" << std::endl;
            std::cout << "   位置: " << info.position << " (0-4095)" << std::endl;
            std::cout << "   速度: " << info.speed << std::endl;
            std::cout << "   负载: " << info.load << std::endl;
            std::cout << "   电压: " << info.voltage / 10.0 << " V" << std::endl;
            std::cout << "   温度: " << (int)info.temperature << " °C" << std::endl;
            std::cout << "   电流: " << info.current << " mA" << std::endl;
            std::cout << "   运动: " << (info.moving ? "是" : "否") << std::endl;
        }

        std::cout << std::endl;
        std::cout << "4️⃣  尝试解锁舵机..." << std::endl;

        // 先禁用扭矩
        std::cout << "   禁用扭矩..." << std::endl;
        servo.enableTorque(target_id, false);
        usleep(200000); // 200ms

        // 尝试解锁 EPROM
        std::cout << "   解锁 EPROM..." << std::endl;
        int unlock_result = servo.unlockEprom(target_id);
        if (unlock_result == 1) {
            std::cout << "   ✅ EPROM 解锁成功" << std::endl;
        } else {
            std::cout << "   ⚠️  EPROM 解锁失败 (可能已解锁)" << std::endl;
        }
        usleep(200000); // 200ms

        // 锁定 EPROM（恢复保护）
        std::cout << "   锁定 EPROM..." << std::endl;
        servo.lockEprom(target_id);
        usleep(200000); // 200ms

        std::cout << std::endl;
        std::cout << "5️⃣  尝试使能扭矩..." << std::endl;

        int max_retries = 3;
        bool success = false;

        for (int retry = 0; retry < max_retries; retry++) {
            if (retry > 0) {
                std::cout << "   重试 " << retry + 1 << "/" << max_retries << "..." << std::endl;
                usleep(200000); // 200ms
            }

            int result = servo.enableTorque(target_id, true);
            // 注意: Ack() 返回 1=成功, 0=失败
            if (result == 1) {
                std::cout << "   ✅ 扭矩使能成功!" << std::endl;
                success = true;
                break;
            } else if (result == 0) {
                std::cout << "   ❌ 扭矩使能失败 (通信错误)" << std::endl;
            } else {
                std::cout << "   ⚠️  返回值异常: " << result << std::endl;
            }
        }

        if (!success) {
            std::cout << std::endl;
            std::cout << "❌ 舵机 " << target_id << " 无法使能扭矩" << std::endl;
            std::cout << std::endl;
            std::cout << "可能原因:" << std::endl;
            std::cout << "  1. 舵机硬件故障" << std::endl;
            std::cout << "  2. 舵机处于错误保护状态" << std::endl;
            std::cout << "  3. 供电不足" << std::endl;
            std::cout << "  4. 机械卡滞" << std::endl;
            std::cout << std::endl;
            std::cout << "建议:" << std::endl;
            std::cout << "  - 检查舵机供电" << std::endl;
            std::cout << "  - 检查机械是否卡滞" << std::endl;
            std::cout << "  - 尝试重新上电" << std::endl;
            return 1;
        }

        std::cout << std::endl;
        std::cout << "6️⃣  测试运动..." << std::endl;

        // 读取当前位置
        int current_pos = servo.readPosition(target_id);
        std::cout << "   当前位置: " << current_pos << std::endl;

        // 小幅度测试运动
        int test_pos = (current_pos < 2048) ? (current_pos + 200) : (current_pos - 200);
        std::cout << "   移动到: " << test_pos << std::endl;

        int move_result = servo.writePosition(target_id, test_pos, 1200, 50);
        if (move_result == 1) {
            std::cout << "   ✅ 命令发送成功" << std::endl;
            sleep(2);

            // 读取新位置
            int new_pos = servo.readPosition(target_id);
            std::cout << "   新位置: " << new_pos << std::endl;

            if (abs(new_pos - test_pos) < 100) {
                std::cout << "   ✅ 舵机运动正常!" << std::endl;
            } else {
                std::cout << "   ⚠️  位置偏差较大" << std::endl;
            }
        } else {
            std::cout << "   ❌ 移动命令失败" << std::endl;
        }

        // 回到中位
        std::cout << "   回到中位 (2048)..." << std::endl;
        servo.writePosition(target_id, 2048, 1200, 50);
        sleep(2);

    } else {
        // 测试所有舵机
        std::cout << "3️⃣  测试所有舵机的扭矩使能..." << std::endl;
        for (auto id : servos) {
            std::cout << std::endl;
            std::cout << "舵机 ID " << (int)id << ":" << std::endl;

            // 先禁用扭矩（重置状态）
            std::cout << "  禁用扭矩..." << std::endl;
            servo.enableTorque(id, false);
            usleep(50000); // 50ms

            // 尝试使能扭矩
            std::cout << "  使能扭矩..." << std::endl;
            int result = servo.enableTorque(id, true);
            // 注意: Ack() 返回 1=成功, 0=失败
            if (result == 1) {
                std::cout << "  ✅ 扭矩使能成功" << std::endl;
            } else {
                std::cout << "  ❌ 扭矩使能失败!" << std::endl;
                std::cout << "  运行: " << argv[0] << " " << port << " " << (int)id << std::endl;
                std::cout << "  进行详细诊断" << std::endl;
            }
        }
    }

    std::cout << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "诊断完成" << std::endl;
    std::cout << "=========================================" << std::endl;

    servo.close();
    return 0;
}
