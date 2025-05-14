/**
 * @file basic_control_example.cpp
 * @brief 基本车辆控制示例程序
 * 
 * 本示例展示了如何使用底盘控制系统的各个模块进行车辆控制。
 * 流程包括：
 * 1. 加载车辆配置
 * 2. 初始化各模块
 * 3. 运行简单的控制循环
 * 4. 发送控制命令到CAN总线
 */

#include "data_input/vehicle_config.h"
#include "data_input/state_estimator.h"
#include "vehicle_control/vehicle_model.h"
#include "vehicle_control/torque_allocator.h"
#include "vehicle_control/torque_smoother.h"
#include "data_output/can_bus_interface.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <csignal>

// 全局变量，用于处理程序终止信号
volatile sig_atomic_t gRunning = 1;

// 信号处理函数
void signalHandler(int signal) {
    std::cout << "接收到终止信号 " << signal << std::endl;
    gRunning = 0;
}

int main(int argc, char* argv[]) {
    std::cout << "底盘控制系统示例程序" << std::endl;
    
    // 设置信号处理函数
    std::signal(SIGINT, signalHandler);  // Ctrl+C
    std::signal(SIGTERM, signalHandler); // 终止信号
    
    // 加载配置文件
    std::string configFile = "config/vehicle_config.yaml";
    if (argc > 1) {
        configFile = argv[1];
    }
    
    std::cout << "加载配置: " << configFile << std::endl;
    
    // 加载车辆配置
    auto vehicleConfig = std::make_shared<chassis_control::data_input::VehicleConfig>();
    if (!vehicleConfig->loadFromFile(configFile)) {
        std::cerr << "无法加载配置文件，使用默认参数" << std::endl;
    }
    
    // 初始化状态估计器
    auto stateEstimator = std::make_shared<chassis_control::data_input::StateEstimator>();
    stateEstimator->setFilterParams(0.1f, 0.2f);  // 设置滤波参数
    
    // 初始化车辆模型
    auto vehicleModel = std::make_shared<chassis_control::vehicle_control::VehicleModel>(vehicleConfig);
    
    // 初始化扭矩分配器
    chassis_control::vehicle_control::TorqueAllocator torqueAllocator(*vehicleConfig);
    torqueAllocator.setAllocateMethod(chassis_control::vehicle_control::AllocateMethod::DYNAMIC_WEIGHT);
    
    // 初始化扭矩平滑器
    chassis_control::vehicle_control::TorqueSmoother torqueSmoother(
        chassis_control::vehicle_control::SmootherType::LOW_PASS_FILTER);
    torqueSmoother.setFilterAlpha(0.2f);  // 设置低通滤波器系数
    
    // 初始化CAN总线接口
    chassis_control::data_output::CANBusInterface canBus;
    bool canInitialized = canBus.initialize("can0");
    
    if (!canInitialized) {
        std::cerr << "警告: CAN总线初始化失败，将模拟输出" << std::endl;
    }
    
    // 主控制循环
    const int cycleTimeMs = 10;  // 控制周期(10ms = 100Hz)
    float targetSpeed = 0.0f;    // 目标速度(m/s)
    float targetAccel = 0.0f;    // 目标加速度(m/s^2)
    float steeringAngle = 0.0f;  // 方向盘转角(rad)
    
    std::cout << "控制循环开始，按Ctrl+C停止..." << std::endl;
    
    while (gRunning) {
        auto cycleStart = std::chrono::steady_clock::now();
        
        // 1. 模拟传感器输入并更新状态估计
        // 在实际系统中，这一步将从真实传感器获取数据
        float rawSpeed = targetSpeed + (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.2f;  // 添加噪声
        float rawAccel = targetAccel + (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.3f;  // 添加噪声
        
        // 更新速度和加速度
        stateEstimator->updateSpeed(rawSpeed);
        stateEstimator->updateAcceleration(rawAccel);
        
        // 更新姿态(模拟)
        std::vector<float> attitude = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // 假设平坦路面
        if (steeringAngle != 0.0f) {
            // 简单估算航向角速度
            attitude[5] = targetSpeed * std::tan(steeringAngle) / vehicleConfig->getWheelBase();
        }
        stateEstimator->updateAttitude(attitude);
        
        // 2. 目标状态更新(这里简单模拟)
        // 在实际系统中，这些值将从车辆控制算法或远程控制获取
        if (targetSpeed < 5.0f && gRunning > 50) {  // 50个周期后开始加速
            targetAccel = 0.5f;  // 加速度0.5 m/s²
        } else if (targetSpeed >= 5.0f) {
            targetAccel = 0.0f;  // 保持恒速
        }
        
        targetSpeed += targetAccel * cycleTimeMs / 1000.0f;
        
        // 简单的S形转向控制
        static int steeringCounter = 0;
        steeringCounter++;
        if (steeringCounter > 200 && steeringCounter < 400) {  // 2-4秒：左转
            steeringAngle = 0.2f;
        } else if (steeringCounter >= 400 && steeringCounter < 600) {  // 4-6秒：右转
            steeringAngle = -0.2f;
        } else {
            steeringAngle = 0.0f;  // 直行
        }
        
        // 3. 控制指令计算
        // 创建控制指令
        chassis_control::vehicle_control::ControlCommand command;
        command.targetSpeed = targetSpeed;
        command.targetAcceleration = targetAccel;
        command.steeringAngle = steeringAngle;
        command.targetYawRate = attitude[5];  // 使用估计的航向角速度
        
        // 更新车辆模型
        if (!vehicleModel->update(command, stateEstimator)) {
            std::cerr << "车辆模型更新失败" << std::endl;
        }
        
        // 获取车轮状态
        auto wheelStates = vehicleModel->getAllWheelStates();
        
        // 4. 扭矩分配
        // 将目标加速度转换为扭矩
        std::vector<float> allocatedTorques = torqueAllocator.allocate(wheelStates, targetAccel);
        
        // 5. 扭矩平滑
        std::vector<float> smoothedTorques = torqueSmoother.smoothTorques(allocatedTorques);
        
        // 6. 发送到CAN总线
        if (canInitialized) {
            canBus.sendTorqueCommands(smoothedTorques);
        } else {
            // 打印部分数据用于调试
            if (steeringCounter % 100 == 0) {  // 每100个周期打印一次
                std::cout << "速度: " << stateEstimator->getFilteredSpeed() 
                          << " m/s, 加速度: " << stateEstimator->getFilteredAcceleration()
                          << " m/s², 转向角: " << steeringAngle << " rad" << std::endl;
                
                std::cout << "扭矩分配: ";
                for (size_t i = 0; i < smoothedTorques.size(); ++i) {
                    std::cout << smoothedTorques[i] << " ";
                }
                std::cout << std::endl;
            }
        }
        
        // 控制循环时间控制
        auto cycleEnd = std::chrono::steady_clock::now();
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(cycleEnd - cycleStart).count();
        
        if (elapsedMs < cycleTimeMs) {
            std::this_thread::sleep_for(std::chrono::milliseconds(cycleTimeMs - elapsedMs));
        } else if (steeringCounter % 100 == 0) {
            std::cerr << "警告: 控制周期超时 " << elapsedMs << "ms" << std::endl;
        }
    }
    
    // 程序终止前安全停止
    std::cout << "程序停止，发送紧急停止命令..." << std::endl;
    
    if (canInitialized) {
        canBus.emergencyStop();
    }
    
    std::cout << "程序已安全退出" << std::endl;
    return 0;
} 