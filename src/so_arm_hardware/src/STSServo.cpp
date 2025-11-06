/*
 * STSServo.cpp
 * 飞特STS系列舵机驱动类实现 (1M波特率专用)
 * 日期: 2025.10.31
 * 作者: Custom Driver
 */

#include "STSServo.h"
#include <unistd.h>
#include <iostream>

// ========== 构造函数与析构函数 ==========

STSServo::STSServo() : initialized(false), currentBaudRate(1000000) {
}

STSServo::~STSServo() {
    if (initialized) {
        close();
    }
}

// ========== 基础连接管理 ==========

bool STSServo::init(const char* serialPort, int baudRate) {
    portName = serialPort;
    currentBaudRate = baudRate;

    if (!servo.begin(baudRate, serialPort)) {
        std::cerr << "STSServo: 无法初始化串口 " << serialPort
                  << " 波特率: " << baudRate << std::endl;
        initialized = false;
        return false;
    }

    initialized = true;
    return true;
}

void STSServo::close() {
    if (initialized) {
        servo.end();
        initialized = false;
    }
}

// ========== 舵机检测与扫描 ==========

int STSServo::ping(u8 id) {
    if (!initialized) return -1;
    return servo.Ping(id);
}

std::vector<u8> STSServo::scanServos(u8 startID, u8 endID) {
    std::vector<u8> foundServos;

    if (!initialized) {
        std::cerr << "STSServo: 未初始化，无法扫描舵机" << std::endl;
        return foundServos;
    }

    for (u8 id = startID; id <= endID; id++) {
        int result = servo.Ping(id);
        if (result != -1) {
            foundServos.push_back(id);
        }
    }

    return foundServos;
}

// ========== 单个舵机位置控制 ==========

int STSServo::writePosition(u8 id, s16 position, u16 speed, u8 acceleration) {
    if (!initialized) return -1;

    // 位置范围检查
    if (position < 0) position = 0;
    if (position > 4095) position = 4095;

    return servo.WritePosEx(id, position, speed, acceleration);
}

int STSServo::regWritePosition(u8 id, s16 position, u16 speed, u8 acceleration) {
    if (!initialized) return -1;

    // 位置范围检查
    if (position < 0) position = 0;
    if (position > 4095) position = 4095;

    return servo.RegWritePosEx(id, position, speed, acceleration);
}

// ========== 多舵机同步控制 ==========

void STSServo::syncWritePosition(u8 ids[], s16 positions[], u16 speeds[],
                                  u8 accelerations[], u8 count) {
    if (!initialized || count == 0) return;

    servo.SyncWritePosEx(ids, count, positions, speeds, accelerations);
}

// ========== 速度控制 (轮式模式) ==========

int STSServo::setServoMode(u8 id) {
    if (!initialized) return -1;
    return servo.ServoMode(id);
}

int STSServo::setWheelMode(u8 id) {
    if (!initialized) return -1;
    return servo.WheelMode(id);
}

int STSServo::writeSpeed(u8 id, s16 speed, u8 acceleration) {
    if (!initialized) return -1;
    return servo.WriteSpe(id, speed, acceleration);
}

// ========== 扭矩控制 ==========

int STSServo::enableTorque(u8 id, bool enable) {
    if (!initialized) return -1;
    return servo.EnableTorque(id, enable ? 1 : 0);
}

// ========== 读取舵机状态 ==========

bool STSServo::readServoInfo(u8 id, ServoInfo& info) {
    if (!initialized) return false;

    // 读取反馈数据
    int result = servo.FeedBack(id);
    if (result == -1) {
        return false;
    }

    // 填充舵机信息
    info.id = id;
    info.position = servo.ReadPos(-1);       // -1表示使用上次FeedBack的数据
    info.speed = servo.ReadSpeed(-1);
    info.load = servo.ReadLoad(-1);
    info.voltage = servo.ReadVoltage(-1);
    info.temperature = servo.ReadTemper(-1);
    info.current = servo.ReadCurrent(-1);
    info.moving = servo.ReadMove(-1);

    return true;
}

int STSServo::readPosition(u8 id) {
    if (!initialized) return -1;
    return servo.ReadPos(id);
}

int STSServo::readSpeed(u8 id) {
    if (!initialized) return -1;
    return servo.ReadSpeed(id);
}

int STSServo::readLoad(u8 id) {
    if (!initialized) return -1;
    return servo.ReadLoad(id);
}

int STSServo::readVoltage(u8 id) {
    if (!initialized) return -1;
    return servo.ReadVoltage(id);
}

int STSServo::readTemperature(u8 id) {
    if (!initialized) return -1;
    return servo.ReadTemper(id);
}

int STSServo::readMoving(u8 id) {
    if (!initialized) return -1;
    return servo.ReadMove(id);
}

int STSServo::readCurrent(u8 id) {
    if (!initialized) return -1;
    return servo.ReadCurrent(id);
}

// ========== EPROM操作 ==========

int STSServo::unlockEprom(u8 id) {
    if (!initialized) return -1;
    return servo.unLockEprom(id);
}

int STSServo::lockEprom(u8 id) {
    if (!initialized) return -1;
    return servo.LockEprom(id);
}

int STSServo::calibrationOffset(u8 id) {
    if (!initialized) return -1;
    return servo.CalibrationOfs(id);
}

// ========== 高级功能 ==========

int STSServo::action(u8 id) {
    if (!initialized) return -1;
    return servo.RegWriteAction(id);
}

bool STSServo::waitMoveDone(u8 id, u32 timeout) {
    if (!initialized) return false;

    u32 startTime = 0;
    u32 elapsed = 0;

    while (elapsed < timeout) {
        int moving = readMoving(id);
        if (moving == 0) {
            return true;  // 舵机停止运动
        }
        if (moving == -1) {
            return false;  // 读取失败
        }

        usleep(10000);  // 等待10ms
        elapsed += 10;
    }

    return false;  // 超时
}

u8 STSServo::getLastError() {
    return servo.getLastError();
}
