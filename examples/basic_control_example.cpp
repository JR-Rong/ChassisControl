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

#include "../chassis_control.h" // 修正头文件路径
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <csignal>
#ifdef _WIN32
#include <windows.h>

#endif

// 全局变量，用于处理程序终止信号
volatile sig_atomic_t gRunning = 1;

int main(int argc, char* argv[]) {
    #ifdef _WIN32
    // 设置控制台输出编码为UTF-8
    SetConsoleOutputCP(CP_UTF8);
    #endif

    std::string config_file = "config.json"; // 配置文件路径
    std::string can_channel = "can0"; // CAN通道
    VehicleParam vehicle_param;
    VehicleState vehicle_state;
    DynamicParam vehicle_dynamic_param;
    
    bool LoadConfigSuccess = LoadConfig(config_file);
    bool InitializeSuccess = Initialize(vehicle_param, can_channel);
    bool UpdateVehicleStateSuccess = UpdateVehicleState(vehicle_state);
    UpdateDynamicParam(vehicle_dynamic_param);
    std::cout << "程序已安全退出" << std::endl;
    return 0;
} 