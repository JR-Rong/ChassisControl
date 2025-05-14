/**
 * @file can_bus_interface.cpp
 * @brief CAN总线通信接口类实现
 */

#include "data_output/can_bus_interface.h"
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>

// Linux下的SocketCAN相关头文件，实际使用时可能需要根据平台调整
#ifdef __linux__
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#endif

namespace chassis_control {
namespace data_output {

CANBusInterface::CANBusInterface()
    : canSocket_(-1),
      channelName_("can0"),
      protocolType_(CANProtocolType::STANDARD_CAN),
      controlMode_(MotorControlMode::TORQUE_MODE),
      isInitialized_(false),
      torqueScale_(10.0f),  // 默认比例因子：10倍
      speedScale_(1.0f) {
    
    // 设置默认电机ID
    motorIDs_ = {0x101, 0x102, 0x103, 0x104, 0x105, 0x106};
}

CANBusInterface::CANBusInterface(const std::string& channel, CANProtocolType protocol)
    : canSocket_(-1),
      channelName_(channel),
      protocolType_(protocol),
      controlMode_(MotorControlMode::TORQUE_MODE),
      isInitialized_(false),
      torqueScale_(10.0f),
      speedScale_(1.0f) {
    
    // 设置默认电机ID
    motorIDs_ = {0x101, 0x102, 0x103, 0x104, 0x105, 0x106};
}

CANBusInterface::~CANBusInterface() {
    close();
}

bool CANBusInterface::initialize(const std::string& channel, int bitrate) {
    // 如果已初始化，先关闭
    if (isInitialized_) {
        close();
    }
    
    channelName_ = channel;
    
#ifdef __linux__
    // 创建SocketCAN套接字
    canSocket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (canSocket_ < 0) {
        std::cerr << "错误: 无法创建CAN套接字" << std::endl;
        return false;
    }
    
    // 指定CAN接口
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, channelName_.c_str(), IFNAMSIZ - 1);
    if (ioctl(canSocket_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "错误: 无法找到CAN接口: " << channelName_ << std::endl;
        close();
        return false;
    }
    
    // 绑定套接字到CAN接口
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(canSocket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "错误: 无法绑定CAN套接字到接口: " << channelName_ << std::endl;
        close();
        return false;
    }
    
    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = 1;  // 1秒超时
    tv.tv_usec = 0;
    setsockopt(canSocket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    // 关闭回环模式（不接收自己发送的消息）
    int loopback = 0;
    setsockopt(canSocket_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    
    // 注意：设置比特率通常在操作系统级别完成，这里不处理
    // 使用其他工具如 ip link set can0 up type can bitrate 500000
    
    isInitialized_ = true;
    return true;
#else
    // 非Linux平台，需要使用其他CAN库
    std::cerr << "警告: 当前平台不支持SocketCAN，需要使用其他CAN库" << std::endl;
    
    // 这里可以添加Windows或其他平台的CAN初始化代码
    // 例如使用Vector XL、PCAN等第三方库
    
    // 模拟初始化成功
    isInitialized_ = true;
    return true;
#endif
}

void CANBusInterface::close() {
#ifdef __linux__
    if (canSocket_ >= 0) {
        ::close(canSocket_);
        canSocket_ = -1;
    }
#else
    // 关闭其他平台的CAN接口
#endif
    
    isInitialized_ = false;
}

bool CANBusInterface::sendTorqueCommands(const std::vector<float>& torques) {
    if (!isInitialized_) {
        std::cerr << "错误: CAN接口未初始化" << std::endl;
        return false;
    }
    
    // 检查扭矩数组大小
    size_t wheelCount = std::min(torques.size(), motorIDs_.size());
    
    if (wheelCount == 0) {
        std::cerr << "错误: 扭矩数组为空" << std::endl;
        return false;
    }
    
    bool success = true;
    
    // 为每个电机发送扭矩命令
    for (size_t i = 0; i < wheelCount; ++i) {
        CANFrame frame;
        frame.id = motorIDs_[i];
        frame.dlc = 8;  // 使用8字节数据长度
        
        // 清空数据字段
        std::memset(frame.data, 0, sizeof(frame.data));
        
        // 设置控制模式字节（假设为第一个字节）
        frame.data[0] = static_cast<uint8_t>(controlMode_);
        
        // 转换扭矩值为CAN数据（假设使用字节1-4）
        convertTorqueToCAN(torques[i], &frame.data[1], torqueScale_);
        
        // 发送CAN帧
        if (!sendCANFrame(frame)) {
            std::cerr << "错误: 发送电机" << i << "扭矩命令失败" << std::endl;
            success = false;
        }
        
        // 小延时，避免CAN总线负载过高
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    return success;
}

bool CANBusInterface::sendSpeedCommands(const std::vector<float>& speeds) {
    if (!isInitialized_) {
        std::cerr << "错误: CAN接口未初始化" << std::endl;
        return false;
    }
    
    // 检查速度数组大小
    size_t wheelCount = std::min(speeds.size(), motorIDs_.size());
    
    if (wheelCount == 0) {
        std::cerr << "错误: 速度数组为空" << std::endl;
        return false;
    }
    
    bool success = true;
    
    // 首先切换到速度控制模式
    MotorControlMode oldMode = controlMode_;
    setMotorControlMode(MotorControlMode::SPEED_MODE);
    
    // 为每个电机发送速度命令
    for (size_t i = 0; i < wheelCount; ++i) {
        CANFrame frame;
        frame.id = motorIDs_[i];
        frame.dlc = 8;
        
        // 清空数据字段
        std::memset(frame.data, 0, sizeof(frame.data));
        
        // 设置控制模式字节
        frame.data[0] = static_cast<uint8_t>(controlMode_);
        
        // 转换速度值为CAN数据
        convertSpeedToCAN(speeds[i], &frame.data[1], speedScale_);
        
        // 发送CAN帧
        if (!sendCANFrame(frame)) {
            std::cerr << "错误: 发送电机" << i << "速度命令失败" << std::endl;
            success = false;
        }
        
        // 小延时
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    // 恢复原控制模式
    setMotorControlMode(oldMode);
    
    return success;
}

bool CANBusInterface::setMotorControlMode(MotorControlMode mode) {
    controlMode_ = mode;
    
    // 发送模式切换命令到所有电机
    bool success = true;
    
    for (size_t i = 0; i < motorIDs_.size(); ++i) {
        CANFrame frame;
        frame.id = motorIDs_[i];
        frame.dlc = 8;
        
        // 清空数据字段
        std::memset(frame.data, 0, sizeof(frame.data));
        
        // 设置控制模式字节
        frame.data[0] = static_cast<uint8_t>(controlMode_);
        
        // 发送CAN帧
        if (!sendCANFrame(frame)) {
            std::cerr << "错误: 发送电机" << i << "控制模式切换命令失败" << std::endl;
            success = false;
        }
        
        // 小延时
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    return success;
}

CANFrame CANBusInterface::receiveCANFrame(int timeout_ms) {
    CANFrame frame;
    
    if (!isInitialized_) {
        std::cerr << "错误: CAN接口未初始化" << std::endl;
        return frame;
    }
    
#ifdef __linux__
    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(canSocket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    // 接收CAN帧
    struct can_frame canFrame;
    int nbytes = read(canSocket_, &canFrame, sizeof(struct can_frame));
    
    if (nbytes > 0) {
        // 转换为我们的CANFrame格式
        frame.id = canFrame.can_id & CAN_EFF_MASK;
        frame.isExtended = (canFrame.can_id & CAN_EFF_FLAG) != 0;
        frame.isRemote = (canFrame.can_id & CAN_RTR_FLAG) != 0;
        frame.dlc = canFrame.can_dlc;
        std::memcpy(frame.data, canFrame.data, std::min(frame.dlc, static_cast<uint8_t>(8)));
    }
#else
    // 其他平台的CAN接收代码
    // ...
    
    // 模拟接收超时
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
#endif
    
    return frame;
}

bool CANBusInterface::sendCANFrame(const CANFrame& frame) {
    if (!isInitialized_) {
        std::cerr << "错误: CAN接口未初始化" << std::endl;
        return false;
    }
    
#ifdef __linux__
    struct can_frame canFrame;
    
    // 设置CAN ID
    canFrame.can_id = frame.id;
    if (frame.isExtended) {
        canFrame.can_id |= CAN_EFF_FLAG;
    }
    if (frame.isRemote) {
        canFrame.can_id |= CAN_RTR_FLAG;
    }
    
    // 设置数据长度
    canFrame.can_dlc = std::min(frame.dlc, static_cast<uint8_t>(8));
    
    // 复制数据
    std::memcpy(canFrame.data, frame.data, canFrame.can_dlc);
    
    // 发送
    int nbytes = write(canSocket_, &canFrame, sizeof(struct can_frame));
    return nbytes == sizeof(struct can_frame);
#else
    // 其他平台的CAN发送代码
    // ...
    
    // 模拟发送成功
    return true;
#endif
}

void CANBusInterface::setMotorIDs(const std::vector<uint32_t>& motor_ids) {
    // 复制电机ID到内部数组
    size_t count = std::min(motor_ids.size(), motorIDs_.size());
    for (size_t i = 0; i < count; ++i) {
        motorIDs_[i] = motor_ids[i];
    }
}

bool CANBusInterface::emergencyStop() {
    if (!isInitialized_) {
        std::cerr << "错误: CAN接口未初始化" << std::endl;
        return false;
    }
    
    bool success = true;
    
    // 为每个电机发送紧急停止命令
    for (size_t i = 0; i < motorIDs_.size(); ++i) {
        CANFrame frame;
        frame.id = motorIDs_[i];
        frame.dlc = 8;
        
        // 清空数据字段
        std::memset(frame.data, 0, sizeof(frame.data));
        
        // 设置紧急停止命令（假设为0xFF）
        frame.data[0] = 0xFF;
        
        // 发送CAN帧
        if (!sendCANFrame(frame)) {
            std::cerr << "错误: 发送电机" << i << "紧急停止命令失败" << std::endl;
            success = false;
        }
        
        // 小延时
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    return success;
}

std::vector<std::array<float, 4>> CANBusInterface::getMotorStatus() {
    std::vector<std::array<float, 4>> status(motorIDs_.size());
    
    if (!isInitialized_) {
        std::cerr << "错误: CAN接口未初始化" << std::endl;
        return status;
    }
    
    // 为每个电机请求状态
    for (size_t i = 0; i < motorIDs_.size(); ++i) {
        CANFrame requestFrame;
        requestFrame.id = motorIDs_[i];
        requestFrame.dlc = 8;
        requestFrame.isRemote = true;  // 使用远程帧请求数据
        
        // 发送请求
        if (!sendCANFrame(requestFrame)) {
            std::cerr << "错误: 发送电机" << i << "状态请求失败" << std::endl;
            continue;
        }
        
        // 接收响应
        CANFrame responseFrame = receiveCANFrame(100);  // 100ms超时
        
        // 解析响应（假设格式：温度、电流、速度、位置）
        if (responseFrame.id == motorIDs_[i]) {
            status[i][0] = parseFloatFromCAN(&responseFrame.data[0], 10.0f);  // 温度
            status[i][1] = parseFloatFromCAN(&responseFrame.data[2], 10.0f);  // 电流
            status[i][2] = parseFloatFromCAN(&responseFrame.data[4], 10.0f);  // 速度
            status[i][3] = parseFloatFromCAN(&responseFrame.data[6], 10.0f);  // 位置
        }
    }
    
    return status;
}

bool CANBusInterface::isConnected() const {
    return isInitialized_;
}

void CANBusInterface::convertTorqueToCAN(float torque, uint8_t* data, float scale) {
    // 将浮点数扭矩转换为整数，考虑比例因子
    int16_t torqueValue = static_cast<int16_t>(torque * scale);
    
    // 分解为两个字节（小端序）
    data[0] = static_cast<uint8_t>(torqueValue & 0xFF);
    data[1] = static_cast<uint8_t>((torqueValue >> 8) & 0xFF);
}

void CANBusInterface::convertSpeedToCAN(float speed, uint8_t* data, float scale) {
    // 将浮点数速度转换为整数，考虑比例因子
    int16_t speedValue = static_cast<int16_t>(speed * scale);
    
    // 分解为两个字节（小端序）
    data[0] = static_cast<uint8_t>(speedValue & 0xFF);
    data[1] = static_cast<uint8_t>((speedValue >> 8) & 0xFF);
}

float CANBusInterface::parseFloatFromCAN(const uint8_t* data, float scale) {
    // 从两个字节重构整数值（小端序）
    int16_t value = static_cast<int16_t>(data[0] | (data[1] << 8));
    
    // 转换为浮点数，考虑比例因子
    return static_cast<float>(value) / scale;
}

} // namespace data_output
} // namespace chassis_control 