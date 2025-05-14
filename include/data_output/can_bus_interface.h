/**
 * @file can_bus_interface.h
 * @brief CAN总线通信接口类
 */

#ifndef CHASSIS_CONTROL_CAN_BUS_INTERFACE_H
#define CHASSIS_CONTROL_CAN_BUS_INTERFACE_H

#include <vector>
#include <string>
#include <array>
#include <cstdint>

namespace chassis_control {
namespace data_output {

/**
 * @struct CANFrame
 * @brief CAN帧结构体
 */
struct CANFrame {
    uint32_t id;                    ///< CAN帧ID
    uint8_t data[8];                ///< 数据字段(最多8字节)
    uint8_t dlc;                    ///< 数据长度代码(0-8)
    bool isExtended;                ///< 是否为扩展帧
    bool isRemote;                  ///< 是否为远程帧
    
    CANFrame() : id(0), dlc(0), isExtended(false), isRemote(false) {
        for (int i = 0; i < 8; ++i) {
            data[i] = 0;
        }
    }
};

/**
 * @enum CANProtocolType
 * @brief CAN协议类型枚举
 */
enum class CANProtocolType {
    STANDARD_CAN,                   ///< 标准CAN
    CAN_FD,                         ///< CAN-FD
    J1939                           ///< J1939协议
};

/**
 * @enum MotorControlMode
 * @brief 电机控制模式枚举
 */
enum class MotorControlMode {
    TORQUE_MODE,                    ///< 扭矩控制模式
    SPEED_MODE,                     ///< 速度控制模式
    POSITION_MODE                   ///< 位置控制模式
};

/**
 * @class CANBusInterface
 * @brief CAN总线通信接口类，负责与车辆硬件通信
 */
class CANBusInterface {
public:
    /**
     * @brief 默认构造函数
     */
    CANBusInterface();
    
    /**
     * @brief 构造函数
     * @param channel CAN通道名称
     * @param protocol CAN协议类型
     */
    CANBusInterface(const std::string& channel, CANProtocolType protocol);
    
    /**
     * @brief 析构函数
     */
    ~CANBusInterface();
    
    /**
     * @brief 初始化CAN接口
     * @param channel CAN通道名称
     * @param bitrate 比特率(bps)
     * @return 是否成功
     */
    bool initialize(const std::string& channel, int bitrate = 500000);
    
    /**
     * @brief 关闭CAN接口
     */
    void close();
    
    /**
     * @brief 发送扭矩命令
     * @param torques 各轮扭矩数组(Nm)
     * @return 是否发送成功
     */
    bool sendTorqueCommands(const std::vector<float>& torques);
    
    /**
     * @brief 发送速度命令
     * @param speeds 各轮速度数组(rpm)
     * @return 是否发送成功
     */
    bool sendSpeedCommands(const std::vector<float>& speeds);
    
    /**
     * @brief 设置电机控制模式
     * @param mode 控制模式
     * @return 是否设置成功
     */
    bool setMotorControlMode(MotorControlMode mode);
    
    /**
     * @brief 接收CAN消息
     * @param timeout_ms 超时时间(毫秒)
     * @return 接收到的CAN帧
     */
    CANFrame receiveCANFrame(int timeout_ms = 100);
    
    /**
     * @brief 发送原始CAN帧
     * @param frame CAN帧
     * @return 是否发送成功
     */
    bool sendCANFrame(const CANFrame& frame);
    
    /**
     * @brief 设置电机CAN ID
     * @param motor_ids 电机CAN ID数组
     */
    void setMotorIDs(const std::vector<uint32_t>& motor_ids);
    
    /**
     * @brief 紧急停止所有电机
     * @return 是否发送成功
     */
    bool emergencyStop();
    
    /**
     * @brief 获取电机状态
     * @return 电机状态数组(温度、电流等)
     */
    std::vector<std::array<float, 4>> getMotorStatus();
    
    /**
     * @brief 检查CAN总线连接状态
     * @return 是否连接正常
     */
    bool isConnected() const;

private:
    /**
     * @brief 转换浮点数扭矩为CAN数据
     * @param torque 扭矩值(Nm)
     * @param data 输出的数据字节
     * @param scale 比例因子
     */
    void convertTorqueToCAN(float torque, uint8_t* data, float scale = 10.0f);
    
    /**
     * @brief 转换浮点数速度为CAN数据
     * @param speed 速度值(rpm)
     * @param data 输出的数据字节
     * @param scale 比例因子
     */
    void convertSpeedToCAN(float speed, uint8_t* data, float scale = 1.0f);
    
    /**
     * @brief 从CAN数据解析浮点数
     * @param data 输入的数据字节
     * @param scale 比例因子
     * @return 解析后的浮点数值
     */
    float parseFloatFromCAN(const uint8_t* data, float scale = 10.0f);

private:
    int canSocket_;                          ///< CAN套接字句柄
    std::string channelName_;                ///< CAN通道名称
    CANProtocolType protocolType_;           ///< CAN协议类型
    MotorControlMode controlMode_;           ///< 电机控制模式
    bool isInitialized_;                     ///< 是否已初始化
    std::array<uint32_t, 6> motorIDs_;       ///< 电机CAN ID数组
    float torqueScale_;                      ///< 扭矩比例因子
    float speedScale_;                       ///< 速度比例因子
};

} // namespace data_output
} // namespace chassis_control

#endif // CHASSIS_CONTROL_CAN_BUS_INTERFACE_H 