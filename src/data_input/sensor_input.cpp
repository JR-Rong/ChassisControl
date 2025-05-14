/**
 * @file sensor_input.cpp
 * @brief 传感器数据输入类的实现
 */

#include "data_input/sensor_input.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <cstring>
#include <cmath>
#include <thread>
#include <stdexcept>

// 条件编译，Windows平台使用模拟代码，Linux平台尝试使用SocketCAN
#ifdef _WIN32
    #include <winsock2.h>
    #define SOCKET_ERROR (-1)
    #define CAN_SIMULATION_MODE
#else
    #include <unistd.h>
    #include <sys/socket.h>
    #include <sys/ioctl.h>
    #include <net/if.h>
    #include <linux/can.h>
    #include <linux/can/raw.h>
    #include <fcntl.h>
#endif

namespace chassis_control {
namespace data_input {

SensorInput::SensorInput() 
    : canSocket(-1), isCANInitialized(false) {
    // 初始化传感器数据
    sensorData.vehicleSpeed = 0.0f;
    sensorData.steeringAngle = 0.0f;
    sensorData.acceleration = 0.0f;
    sensorData.wheelSpeeds = std::vector<float>(6, 0.0f);
    sensorData.vehicleAttitude = std::vector<float>(6, 0.0f);
    sensorData.timestamp = 0;
    
    // 设置默认CAN解析配置
    setDefaultCANConfig();
}

SensorInput::SensorInput(const std::string& can_interface, int wheel_count) 
    : canSocket(-1), canInterface(can_interface), isCANInitialized(false) {
    // 初始化传感器数据
    sensorData.vehicleSpeed = 0.0f;
    sensorData.steeringAngle = 0.0f;
    sensorData.acceleration = 0.0f;
    sensorData.wheelSpeeds = std::vector<float>(wheel_count, 0.0f);
    sensorData.vehicleAttitude = std::vector<float>(6, 0.0f);
    sensorData.timestamp = 0;
    
    // 设置默认CAN解析配置
    setDefaultCANConfig();
    
    // 尝试初始化CAN接口
    initCANInterface(can_interface);
}

SensorInput::~SensorInput() {
    // 关闭CAN套接字
    if (canSocket >= 0) {
#ifdef _WIN32
        closesocket(canSocket);
#else
        close(canSocket);
#endif
    }
}

bool SensorInput::initCANInterface(const std::string& can_interface) {
    canInterface = can_interface;
    
#ifdef CAN_SIMULATION_MODE
    // 模拟模式下不实际打开CAN接口
    std::cout << "运行在CAN模拟模式，不会实际访问CAN接口" << std::endl;
    isCANInitialized = true;
    return true;
#else
    // 创建原始CAN套接字
    canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (canSocket < 0) {
        std::cerr << "无法创建CAN套接字: " << strerror(errno) << std::endl;
        return false;
    }
    
    // 获取接口索引
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, can_interface.c_str());
    if (ioctl(canSocket, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "无法获取CAN接口索引: " << strerror(errno) << std::endl;
        close(canSocket);
        canSocket = -1;
        return false;
    }
    
    // 绑定套接字到CAN接口
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "无法绑定到CAN接口: " << strerror(errno) << std::endl;
        close(canSocket);
        canSocket = -1;
        return false;
    }
    
    // 设置非阻塞模式
    int flags = fcntl(canSocket, F_GETFL, 0);
    if (flags == -1) {
        std::cerr << "无法获取套接字标志: " << strerror(errno) << std::endl;
        close(canSocket);
        canSocket = -1;
        return false;
    }
    
    flags |= O_NONBLOCK;
    if (fcntl(canSocket, F_SETFL, flags) == -1) {
        std::cerr << "无法设置非阻塞模式: " << strerror(errno) << std::endl;
        close(canSocket);
        canSocket = -1;
        return false;
    }
    
    isCANInitialized = true;
    std::cout << "成功初始化CAN接口: " << can_interface << std::endl;
    return true;
#endif
}

bool SensorInput::updateFromCAN() {
    if (!isCANInitialized) {
        std::cerr << "CAN接口未初始化" << std::endl;
        return false;
    }
    
#ifdef CAN_SIMULATION_MODE
    // 在模拟模式下生成随机数据
    static float simSpeed = 0.0f;
    static float simSteering = 0.0f;
    static float simAccel = 0.0f;
    
    // 简单模拟一些变化
    simSpeed += (std::rand() % 100 - 50) / 1000.0f;  // 随机增减速度
    if (simSpeed < 0.0f) simSpeed = 0.0f;
    if (simSpeed > 30.0f) simSpeed = 30.0f;
    
    simSteering += (std::rand() % 100 - 50) / 1000.0f;  // 随机转向
    if (simSteering < -0.5f) simSteering = -0.5f;
    if (simSteering > 0.5f) simSteering = 0.5f;
    
    simAccel += (std::rand() % 100 - 50) / 1000.0f;  // 随机加减速
    if (simAccel < -5.0f) simAccel = -5.0f;
    if (simAccel > 5.0f) simAccel = 5.0f;
    
    // 更新传感器数据
    sensorData.vehicleSpeed = simSpeed;
    sensorData.steeringAngle = simSteering;
    sensorData.acceleration = simAccel;
    
    // 更新车轮速度
    for (size_t i = 0; i < sensorData.wheelSpeeds.size(); ++i) {
        float wheelSpeedVariation = (std::rand() % 100 - 50) / 1000.0f;
        sensorData.wheelSpeeds[i] = simSpeed / 0.3f + wheelSpeedVariation;  // 假设轮半径为0.3m
    }
    
    // 更新姿态
    sensorData.vehicleAttitude[0] = (std::rand() % 100 - 50) / 1000.0f;  // 横摇角
    sensorData.vehicleAttitude[1] = (std::rand() % 100 - 50) / 1000.0f;  // 俯仰角
    sensorData.vehicleAttitude[2] = simSteering * 0.1f;                  // 偏航角与转向相关
    sensorData.vehicleAttitude[3] = (std::rand() % 100 - 50) / 1000.0f;  // 横摇角速度
    sensorData.vehicleAttitude[4] = (std::rand() % 100 - 50) / 1000.0f;  // 俯仰角速度
    sensorData.vehicleAttitude[5] = (std::rand() % 100 - 50) / 1000.0f;  // 偏航角速度
    
    // 更新时间戳
    sensorData.timestamp = getCurrentTimestamp();
    return true;
#else
    // 实际从CAN总线读取数据
    struct can_frame frame;
    bool dataUpdated = false;
    
    // 从CAN套接字读取所有可用的帧
    while (true) {
        ssize_t nbytes = read(canSocket, &frame, sizeof(struct can_frame));
        
        if (nbytes == SOCKET_ERROR && errno == EAGAIN) {
            // 没有更多数据可读
            break;
        } else if (nbytes != sizeof(struct can_frame)) {
            // 读取错误
            if (nbytes < 0) {
                std::cerr << "读取CAN帧错误: " << strerror(errno) << std::endl;
            } else {
                std::cerr << "接收到不完整的CAN帧" << std::endl;
            }
            continue;
        }
        
        // 转换为我们的消息格式
        CANMessage msg;
        msg.id = frame.can_id;
        msg.dlc = frame.can_dlc;
        memcpy(msg.data, frame.data, sizeof(msg.data));
        msg.timestamp = getCurrentTimestamp();
        
        // 解析CAN消息
        if (parseCANMessage(msg)) {
            dataUpdated = true;
        }
    }
    
    // 如果有数据更新，更新时间戳
    if (dataUpdated) {
        sensorData.timestamp = getCurrentTimestamp();
    }
    
    return dataUpdated;
#endif
}

void SensorInput::updateFromSimulation(float speed, float steering_angle, float accel,
                                     const std::vector<float>& wheel_speeds,
                                     const std::vector<float>& attitude) {
    // 更新传感器数据
    sensorData.vehicleSpeed = speed;
    sensorData.steeringAngle = steering_angle;
    sensorData.acceleration = accel;
    
    // 更新车轮速度
    if (wheel_speeds.size() <= sensorData.wheelSpeeds.size()) {
        std::copy(wheel_speeds.begin(), wheel_speeds.end(), sensorData.wheelSpeeds.begin());
    } else {
        std::cerr << "提供的车轮速度数量超过了配置的车轮数量" << std::endl;
        std::copy(wheel_speeds.begin(), wheel_speeds.begin() + sensorData.wheelSpeeds.size(), 
                 sensorData.wheelSpeeds.begin());
    }
    
    // 更新姿态
    if (attitude.size() <= sensorData.vehicleAttitude.size()) {
        std::copy(attitude.begin(), attitude.end(), sensorData.vehicleAttitude.begin());
    } else {
        std::cerr << "提供的姿态数据数量超过了配置的姿态数据数量" << std::endl;
        std::copy(attitude.begin(), attitude.begin() + sensorData.vehicleAttitude.size(), 
                 sensorData.vehicleAttitude.begin());
    }
    
    // 更新时间戳
    sensorData.timestamp = getCurrentTimestamp();
    
    // 将数据添加到历史记录
    historicalData.push_back(sensorData);
    
    // 控制历史数据大小
    const size_t MAX_HISTORY_SIZE = 1000;
    if (historicalData.size() > MAX_HISTORY_SIZE) {
        historicalData.erase(historicalData.begin());
    }
}

float SensorInput::getVehicleSpeed() const {
    return sensorData.vehicleSpeed;
}

float SensorInput::getSteeringAngle() const {
    return sensorData.steeringAngle;
}

float SensorInput::getAcceleration() const {
    return sensorData.acceleration;
}

const std::vector<float>& SensorInput::getWheelSpeeds() const {
    return sensorData.wheelSpeeds;
}

const std::vector<float>& SensorInput::getVehicleAttitude() const {
    return sensorData.vehicleAttitude;
}

uint64_t SensorInput::getTimestamp() const {
    return sensorData.timestamp;
}

void SensorInput::printSensorData() const {
    std::cout << "======= 传感器数据 =======" << std::endl;
    std::cout << "时间戳: " << sensorData.timestamp << " μs" << std::endl;
    std::cout << "车速: " << sensorData.vehicleSpeed << " m/s" << std::endl;
    std::cout << "方向盘转角: " << sensorData.steeringAngle << " rad" << std::endl;
    std::cout << "加速度: " << sensorData.acceleration << " m/s^2" << std::endl;
    
    std::cout << "--- 车轮速度 ---" << std::endl;
    for (size_t i = 0; i < sensorData.wheelSpeeds.size(); ++i) {
        std::cout << "车轮 " << i << ": " << sensorData.wheelSpeeds[i] << " rad/s" << std::endl;
    }
    
    std::cout << "--- 车辆姿态 ---" << std::endl;
    if (sensorData.vehicleAttitude.size() >= 6) {
        std::cout << "横摇角: " << sensorData.vehicleAttitude[0] << " rad" << std::endl;
        std::cout << "俯仰角: " << sensorData.vehicleAttitude[1] << " rad" << std::endl;
        std::cout << "偏航角: " << sensorData.vehicleAttitude[2] << " rad" << std::endl;
        std::cout << "横摇角速度: " << sensorData.vehicleAttitude[3] << " rad/s" << std::endl;
        std::cout << "俯仰角速度: " << sensorData.vehicleAttitude[4] << " rad/s" << std::endl;
        std::cout << "偏航角速度: " << sensorData.vehicleAttitude[5] << " rad/s" << std::endl;
    }
    std::cout << "==========================" << std::endl;
}

bool SensorInput::saveDataToFile(const std::string& filename) const {
    if (historicalData.empty()) {
        std::cerr << "没有历史数据可保存" << std::endl;
        return false;
    }
    
    try {
        std::ofstream file(filename, std::ios::out | std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "无法打开文件进行写入: " << filename << std::endl;
            return false;
        }
        
        // 写入历史数据数量
        size_t dataCount = historicalData.size();
        file.write(reinterpret_cast<const char*>(&dataCount), sizeof(dataCount));
        
        // 写入车轮数量和姿态数据维度
        size_t wheelCount = historicalData[0].wheelSpeeds.size();
        size_t attitudeCount = historicalData[0].vehicleAttitude.size();
        file.write(reinterpret_cast<const char*>(&wheelCount), sizeof(wheelCount));
        file.write(reinterpret_cast<const char*>(&attitudeCount), sizeof(attitudeCount));
        
        // 写入所有历史数据
        for (const auto& data : historicalData) {
            // 写入基本数据
            file.write(reinterpret_cast<const char*>(&data.vehicleSpeed), sizeof(data.vehicleSpeed));
            file.write(reinterpret_cast<const char*>(&data.steeringAngle), sizeof(data.steeringAngle));
            file.write(reinterpret_cast<const char*>(&data.acceleration), sizeof(data.acceleration));
            file.write(reinterpret_cast<const char*>(&data.timestamp), sizeof(data.timestamp));
            
            // 写入车轮速度数据
            for (const auto& speed : data.wheelSpeeds) {
                file.write(reinterpret_cast<const char*>(&speed), sizeof(speed));
            }
            
            // 写入姿态数据
            for (const auto& attitude : data.vehicleAttitude) {
                file.write(reinterpret_cast<const char*>(&attitude), sizeof(attitude));
            }
        }
        
        file.close();
        std::cout << "成功保存 " << dataCount << " 条传感器数据记录到文件: " << filename << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "保存数据时发生错误: " << e.what() << std::endl;
        return false;
    }
}

bool SensorInput::loadDataFromFile(const std::string& filename) {
    try {
        std::ifstream file(filename, std::ios::in | std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "无法打开文件进行读取: " << filename << std::endl;
            return false;
        }
        
        // 读取历史数据数量
        size_t dataCount = 0;
        file.read(reinterpret_cast<char*>(&dataCount), sizeof(dataCount));
        
        // 读取车轮数量和姿态数据维度
        size_t wheelCount = 0;
        size_t attitudeCount = 0;
        file.read(reinterpret_cast<char*>(&wheelCount), sizeof(wheelCount));
        file.read(reinterpret_cast<char*>(&attitudeCount), sizeof(attitudeCount));
        
        // 清空当前历史数据
        historicalData.clear();
        
        // 读取所有历史数据
        for (size_t i = 0; i < dataCount; ++i) {
            VehicleSensorData data;
            
            // 读取基本数据
            file.read(reinterpret_cast<char*>(&data.vehicleSpeed), sizeof(data.vehicleSpeed));
            file.read(reinterpret_cast<char*>(&data.steeringAngle), sizeof(data.steeringAngle));
            file.read(reinterpret_cast<char*>(&data.acceleration), sizeof(data.acceleration));
            file.read(reinterpret_cast<char*>(&data.timestamp), sizeof(data.timestamp));
            
            // 读取车轮速度数据
            data.wheelSpeeds.resize(wheelCount);
            for (size_t j = 0; j < wheelCount; ++j) {
                file.read(reinterpret_cast<char*>(&data.wheelSpeeds[j]), sizeof(float));
            }
            
            // 读取姿态数据
            data.vehicleAttitude.resize(attitudeCount);
            for (size_t j = 0; j < attitudeCount; ++j) {
                file.read(reinterpret_cast<char*>(&data.vehicleAttitude[j]), sizeof(float));
            }
            
            // 添加到历史数据
            historicalData.push_back(data);
        }
        
        // 如果有历史数据，则将最后一条设置为当前数据
        if (!historicalData.empty()) {
            sensorData = historicalData.back();
        }
        
        file.close();
        std::cout << "成功从文件加载 " << dataCount << " 条传感器数据记录: " << filename << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "加载数据时发生错误: " << e.what() << std::endl;
        return false;
    }
}

bool SensorInput::parseCANMessage(const CANMessage& msg) {
    // 根据CAN ID解析不同类型的消息
    if (msg.id == canConfig.speedID) {
        parseVehicleSpeed(msg);
        return true;
    } 
    else if (msg.id == canConfig.steeringAngleID) {
        parseSteeringAngle(msg);
        return true;
    } 
    else if (msg.id == canConfig.accelerationID) {
        parseAcceleration(msg);
        return true;
    } 
    else if (msg.id == canConfig.attitudeIDs[0] || msg.id == canConfig.attitudeIDs[1]) {
        // 判断是角度还是角速度
        int attitudeIndex = (msg.id == canConfig.attitudeIDs[0]) ? 0 : 1;
        parseAttitude(msg, attitudeIndex);
        return true;
    }
    
    // 检查是否是轮速消息
    for (int i = 0; i < 6; ++i) {
        if (msg.id == canConfig.wheelSpeedIDs[i]) {
            parseWheelSpeed(msg, i);
            return true;
        }
    }
    
    // 未知的CAN ID
    return false;
}

void SensorInput::parseVehicleSpeed(const CANMessage& msg) {
    // 假设车速数据在前两个字节
    uint16_t rawSpeed = (static_cast<uint16_t>(msg.data[0]) << 8) | msg.data[1];
    
    // 转换为m/s，假设原始数据单位为0.01km/h
    float speedKmh = static_cast<float>(rawSpeed) * 0.01f;
    sensorData.vehicleSpeed = speedKmh / 3.6f;  // 转换为m/s
}

void SensorInput::parseSteeringAngle(const CANMessage& msg) {
    // 假设方向盘转角数据在前两个字节，有符号
    int16_t rawAngle = (static_cast<int16_t>(msg.data[0]) << 8) | msg.data[1];
    
    // 转换为弧度，假设原始数据单位为0.1度
    float angleDegrees = static_cast<float>(rawAngle) * 0.1f;
    sensorData.steeringAngle = angleDegrees * M_PI / 180.0f;  // 转换为弧度
}

void SensorInput::parseAcceleration(const CANMessage& msg) {
    // 假设加速度数据在前两个字节，有符号
    int16_t rawAccel = (static_cast<int16_t>(msg.data[0]) << 8) | msg.data[1];
    
    // 转换为m/s^2，假设原始数据单位为0.01g
    float accelG = static_cast<float>(rawAccel) * 0.01f;
    sensorData.acceleration = accelG * 9.81f;  // 转换为m/s^2
}

void SensorInput::parseWheelSpeed(const CANMessage& msg, int wheel_index) {
    // 检查索引是否有效
    if (wheel_index < 0 || wheel_index >= static_cast<int>(sensorData.wheelSpeeds.size())) {
        std::cerr << "无效的车轮索引: " << wheel_index << std::endl;
        return;
    }
    
    // 假设轮速数据在前两个字节
    uint16_t rawSpeed = (static_cast<uint16_t>(msg.data[0]) << 8) | msg.data[1];
    
    // 转换为rad/s，假设原始数据单位为0.01rpm
    float speedRpm = static_cast<float>(rawSpeed) * 0.01f;
    sensorData.wheelSpeeds[wheel_index] = speedRpm * 2.0f * M_PI / 60.0f;  // 转换为rad/s
}

void SensorInput::parseAttitude(const CANMessage& msg, int attitude_part) {
    // 检查是角度还是角速度
    bool isAngle = (attitude_part == 0);
    int baseIndex = isAngle ? 0 : 3;  // 0,1,2为角度，3,4,5为角速度
    
    // 解析三个轴的数据
    for (int i = 0; i < 3; ++i) {
        // 每个轴占用两个字节
        int16_t rawValue = (static_cast<int16_t>(msg.data[i*2]) << 8) | msg.data[i*2+1];
        
        // 转换为弧度或弧度/秒，假设原始数据单位为0.01度或0.01度/秒
        float valueDegrees = static_cast<float>(rawValue) * 0.01f;
        sensorData.vehicleAttitude[baseIndex + i] = valueDegrees * M_PI / 180.0f;
    }
}

uint64_t SensorInput::getCurrentTimestamp() const {
    // 获取当前时间戳（微秒）
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

void SensorInput::setDefaultCANConfig() {
    // 设置默认CAN ID
    canConfig.speedID = 0x100;          // 车速
    canConfig.steeringAngleID = 0x101;  // 方向盘转角
    canConfig.accelerationID = 0x102;   // 加速度
    
    // 6个车轮的轮速ID
    canConfig.wheelSpeedIDs[0] = 0x200;  // 右前轮
    canConfig.wheelSpeedIDs[1] = 0x201;  // 左前轮
    canConfig.wheelSpeedIDs[2] = 0x202;  // 右中轮
    canConfig.wheelSpeedIDs[3] = 0x203;  // 左中轮
    canConfig.wheelSpeedIDs[4] = 0x204;  // 右后轮
    canConfig.wheelSpeedIDs[5] = 0x205;  // 左后轮
    
    // 姿态数据ID
    canConfig.attitudeIDs[0] = 0x300;    // 角度
    canConfig.attitudeIDs[1] = 0x301;    // 角速度
}

} // namespace data_input
} // namespace chassis_control 