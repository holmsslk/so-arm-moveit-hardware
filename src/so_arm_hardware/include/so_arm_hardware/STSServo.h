/*
 * STSServo.h
 * 飞特STS系列舵机驱动类 (1M波特率专用)
 * 日期: 2025.10.31
 * 作者: Custom Driver
 */

#ifndef _STSSERVO_H
#define _STSSERVO_H

#include "SMS_STS.h"
#include <vector>
#include <string>

// 舵机信息结构体
struct ServoInfo {
    u8 id;                  // 舵机ID
    s16 position;           // 当前位置 (0-4095)
    s16 speed;              // 当前速度
    s16 load;               // 当前负载
    u8 voltage;             // 当前电压 (单位: 0.1V)
    u8 temperature;         // 当前温度 (℃)
    s16 current;            // 当前电流 (单位: mA)
    u8 moving;              // 是否在运动 (0:停止, 1:运动)
};

// 舵机运动模式
enum ServoMode {
    SERVO_MODE = 0,         // 位置控制模式
    WHEEL_MODE = 1          // 轮式连续旋转模式
};

// 舵机控制类
class STSServo {
public:
    // 构造函数
    STSServo();
    ~STSServo();

    // ========== 基础连接管理 ==========

    /**
     * @brief 初始化舵机通信
     * @param serialPort 串口设备路径 (例如: "/dev/ttyACM0")
     * @param baudRate 波特率 (默认: 1000000)
     * @return 成功返回true，失败返回false
     */
    bool init(const char* serialPort, int baudRate = 1000000);

    /**
     * @brief 关闭舵机通信
     */
    void close();

    // ========== 舵机检测与扫描 ==========

    /**
     * @brief 检测指定ID的舵机是否在线
     * @param id 舵机ID (1-254)
     * @return 在线返回ID，离线返回-1
     */
    int ping(u8 id);

    /**
     * @brief 扫描总线上所有在线的舵机
     * @param startID 起始ID (默认: 1)
     * @param endID 结束ID (默认: 20)
     * @return 在线舵机ID列表
     */
    std::vector<u8> scanServos(u8 startID = 1, u8 endID = 20);

    // ========== 单个舵机位置控制 ==========

    /**
     * @brief 控制舵机运动到指定位置
     * @param id 舵机ID
     * @param position 目标位置 (0-4095)
     * @param speed 运动速度 (0-4095, 0表示最大速度)
     * @param acceleration 加速度 (0-254, 0表示无加速度控制)
     * @return 成功返回0，失败返回-1
     */
    int writePosition(u8 id, s16 position, u16 speed = 0, u8 acceleration = 0);

    /**
     * @brief 异步写入舵机位置 (需要调用action()执行)
     * @param id 舵机ID
     * @param position 目标位置 (0-4095)
     * @param speed 运动速度
     * @param acceleration 加速度
     * @return 成功返回0，失败返回-1
     */
    int regWritePosition(u8 id, s16 position, u16 speed = 0, u8 acceleration = 0);

    // ========== 多舵机同步控制 ==========

    /**
     * @brief 同步控制多个舵机运动到指定位置
     * @param ids 舵机ID数组
     * @param positions 目标位置数组
     * @param speeds 速度数组 (可为NULL)
     * @param accelerations 加速度数组 (可为NULL)
     * @param count 舵机数量
     */
    void syncWritePosition(u8 ids[], s16 positions[], u16 speeds[] = nullptr,
                          u8 accelerations[] = nullptr, u8 count = 0);

    // ========== 速度控制 (轮式模式) ==========

    /**
     * @brief 设置舵机为位置控制模式
     * @param id 舵机ID
     * @return 成功返回0，失败返回-1
     */
    int setServoMode(u8 id);

    /**
     * @brief 设置舵机为轮式模式
     * @param id 舵机ID
     * @return 成功返回0，失败返回-1
     */
    int setWheelMode(u8 id);

    /**
     * @brief 在轮式模式下控制舵机速度
     * @param id 舵机ID
     * @param speed 速度 (正值正转，负值反转)
     * @param acceleration 加速度
     * @return 成功返回0，失败返回-1
     */
    int writeSpeed(u8 id, s16 speed, u8 acceleration = 0);

    // ========== 扭矩控制 ==========

    /**
     * @brief 启用/禁用舵机扭矩输出
     * @param id 舵机ID
     * @param enable true启用，false禁用
     * @return 成功返回0，失败返回-1
     */
    int enableTorque(u8 id, bool enable);

    // ========== 读取舵机状态 ==========

    /**
     * @brief 读取舵机完整信息
     * @param id 舵机ID
     * @param info 输出舵机信息结构体
     * @return 成功返回true，失败返回false
     */
    bool readServoInfo(u8 id, ServoInfo& info);

    /**
     * @brief 读取舵机当前位置
     * @param id 舵机ID
     * @return 位置值 (0-4095), 失败返回-1
     */
    int readPosition(u8 id);

    /**
     * @brief 读取舵机当前速度
     * @param id 舵机ID
     * @return 速度值, 失败返回-1
     */
    int readSpeed(u8 id);

    /**
     * @brief 读取舵机当前负载
     * @param id 舵机ID
     * @return 负载值 (0-1000), 失败返回-1
     */
    int readLoad(u8 id);

    /**
     * @brief 读取舵机电压
     * @param id 舵机ID
     * @return 电压值 (单位: 0.1V), 失败返回-1
     */
    int readVoltage(u8 id);

    /**
     * @brief 读取舵机温度
     * @param id 舵机ID
     * @return 温度值 (℃), 失败返回-1
     */
    int readTemperature(u8 id);

    /**
     * @brief 读取舵机运动状态
     * @param id 舵机ID
     * @return 0表示停止，1表示运动中，-1表示失败
     */
    int readMoving(u8 id);

    /**
     * @brief 读取舵机电流
     * @param id 舵机ID
     * @return 电流值 (mA), 失败返回-1
     */
    int readCurrent(u8 id);

    // ========== EPROM操作 ==========

    /**
     * @brief 解锁EPROM (允许写入EPROM区域)
     * @param id 舵机ID
     * @return 成功返回0，失败返回-1
     */
    int unlockEprom(u8 id);

    /**
     * @brief 锁定EPROM (保护EPROM区域)
     * @param id 舵机ID
     * @return 成功返回0，失败返回-1
     */
    int lockEprom(u8 id);

    /**
     * @brief 中位校准
     * @param id 舵机ID
     * @return 成功返回0，失败返回-1
     */
    int calibrationOffset(u8 id);

    // ========== 高级功能 ==========

    /**
     * @brief 执行异步写入指令
     * @param id 舵机ID (0xFE表示广播给所有舵机)
     * @return 成功返回0，失败返回-1
     */
    int action(u8 id = 0xFE);

    /**
     * @brief 等待舵机运动完成
     * @param id 舵机ID
     * @param timeout 超时时间 (毫秒)
     * @return 完成返回true，超时返回false
     */
    bool waitMoveDone(u8 id, u32 timeout = 5000);

    /**
     * @brief 获取通信状态错误码
     * @return 错误码
     */
    u8 getLastError();

    /**
     * @brief 检查舵机是否已初始化
     * @return 已初始化返回true
     */
    bool isInitialized() const { return initialized; }

private:
    SMS_STS servo;          // 底层舵机通信对象
    bool initialized;       // 初始化标志
    std::string portName;   // 串口名称
    int currentBaudRate;    // 当前波特率
};

#endif // _STSSERVO_H
