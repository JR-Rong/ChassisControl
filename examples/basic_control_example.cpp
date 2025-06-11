/**
 * @file basic_control_example.cpp
 * @brief 基本车辆控制示例程序
 * 
 * 本示例展示了如何使用底盘控制系统的各个模块进行车辆控制。
 * 主要功能：
 * 1. 加载车辆配置
 * 2. 初始化车辆参数
 * 3. 实时更新车辆状态
 * 4. 执行基本控制命令
 * 5. 发送CAN控制信息
 * 6. 异常处理和安全保护
 */


#include "../chassis_control.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <csignal>
#include <iomanip>
#include <cmath>
#ifdef _WIN32
#include <windows.h>
#endif

// 全局变量，用于处理程序终止信号
volatile sig_atomic_t gRunning = 1;

// 信号处理函数
void signalHandler(int signum) {
    std::cout << "\n接收到终止信号，程序准备退出..." << std::endl;
    gRunning = 0;
}

// 打印分隔线
void printSeparator() {
    std::cout << std::string(50, '-') << std::endl;
}

// 初始化车辆静态参数
StaticParam initializeStaticParam() {
    StaticParam static_param;
    static_param.max_steering_angle = 0.785f;    // 45度
    static_param.max_torque = 500.0f;            // 最大扭矩500Nm
    static_param.max_velocity = 20.0f;           // 最大速度20m/s
    static_param.max_acceleration = 3.0f;        // 最大加速度3m/s²
    static_param.max_deceleration = 5.0f;        // 最大减速度5m/s²
    static_param.wheel_count = 4;                // 四轮车辆
    
    // 初始化轮胎特性曲线（示例数据）
    WheelCharacteristicsCurve curve;
    curve.slip_vs_longitudinal_force = {{0.0, 0.0}, {0.1, 1000.0}, {0.2, 1500.0}};
    curve.slip_angle_vs_lateral_force = {{0.0, 0.0}, {0.1, 800.0}, {0.2, 1200.0}};
    static_param.characteristics_curve["default"] = curve;
    
    return static_param;
}

// 初始化车轮参数
std::vector<WheelParam> initializeWheelParams() {
    std::vector<WheelParam> wheel_params;
    
    // 前轮参数
    for(int i = 0; i < 2; i++) {
        WheelParam wheel;
        wheel.wheel_row = 0;
        wheel.wheel_col = i;
        wheel.feture_name = "default";
        wheel.mu = 0.8f;
        wheel.slip_threshold = 0.2f;
        wheel.wheel_width = 0.245f;
        wheel.wheel_radius = 0.33f;
        wheel.wheel_pressure = 220.0f;  // kPa
        wheel.wheel_position = {i == 0 ? -0.8f : 0.8f, 1.5f, 0.33f};
        wheel_params.push_back(wheel);
    }
    
    // 后轮参数
    for(int i = 0; i < 2; i++) {
        WheelParam wheel;
        wheel.wheel_row = 1;
        wheel.wheel_col = i;
        wheel.feture_name = "default";
        wheel.mu = 0.8f;
        wheel.slip_threshold = 0.2f;
        wheel.wheel_width = 0.245f;
        wheel.wheel_radius = 0.33f;
        wheel.wheel_pressure = 220.0f;  // kPa
        wheel.wheel_position = {i == 0 ? -0.8f : 0.8f, -1.5f, 0.33f};
        wheel_params.push_back(wheel);
    }
    
    return wheel_params;
}

// 初始化车辆动态参数
DynamicParam initializeDynamicParam() {
    DynamicParam dynamic_param;
    dynamic_param.mass = 1500.0f;  // 车重1500kg
    dynamic_param.cog_position = {0.0f, 0.0f, 0.5f};  // 质心位置
    dynamic_param.wheel_params = initializeWheelParams();
    return dynamic_param;
}

// 初始化车辆状态
VehicleState initializeVehicleState() {
    VehicleState state;
    state.cur_velocity = {0.0f, 0.0f};
    state.cur_acceleration = {0.0f, 0.0f};
    state.cur_steering_angle = 0.0f;
    state.target_velocity = {0.0f, 0.0f};
    state.target_acceleration = {0.0f, 0.0f};
    state.target_steering_angle = 0.0f;
    state.target_position = {0.0f, 0.0f, 0.0f};
    state.vehicle_posture = {0.0f, 0.0f, 0.0f};
    state.vehicle_position = {0.0f, 0.0f, 0.0f};
    
    // 初始化轮胎状态
    for(int i = 0; i < 4; i++) {
        WheelState wheel;
        wheel.wheel_row = i / 2;
        wheel.wheel_col = i % 2;
        wheel.steeringAngle = {0.0f, 0.0f};
        wheel.angularVelocity = {0.0f, 0.0f};
        wheel.is_sliding = false;
        wheel.vertical_displacement = 0.0f;
        state.tier_state.push_back(wheel);
    }
    
    return state;
}

// 打印车辆状态
void printVehicleState(const VehicleState& state) {
    std::cout << std::fixed << std::setprecision(2);
    printSeparator();
    std::cout << "【当前车辆状态】" << std::endl;
    std::cout << "速度(纵向/横向): " << state.cur_velocity[0] << "/" << state.cur_velocity[1] << " m/s" << std::endl;
    std::cout << "加速度(纵向/横向): " << state.cur_acceleration[0] << "/" << state.cur_acceleration[1] << " m/s²" << std::endl;
    std::cout << "转向角: " << state.cur_steering_angle * 180.0f / M_PI << " 度" << std::endl;
    std::cout << "位置(x/y/z): " << state.vehicle_position[0] << "/" 
              << state.vehicle_position[1] << "/" << state.vehicle_position[2] << " m" << std::endl;
    
    std::cout << "\n【目标状态】" << std::endl;
    std::cout << "目标速度(纵向/横向): " << state.target_velocity[0] << "/" << state.target_velocity[1] << " m/s" << std::endl;
    std::cout << "目标加速度(纵向/横向): " << state.target_acceleration[0] << "/" << state.target_acceleration[1] << " m/s²" << std::endl;
    std::cout << "目标转向角: " << state.target_steering_angle * 180.0f / M_PI << " 度" << std::endl;
    printSeparator();
}

int main(int argc, char* argv[]) {
    #ifdef _WIN32
    // 设置控制台输出编码为UTF-8
    SetConsoleOutputCP(CP_UTF8);
    #endif
    
    // 注册信号处理函数
    signal(SIGINT, signalHandler);
    
    // 配置文件和CAN通道设置
    std::string config_file = "config.json";
    std::string can_channel = "can0";
    
    std::cout << "正在初始化车辆控制系统..." << std::endl;
    
    // 初始化车辆参数
    VehicleParam vehicle_param;
    vehicle_param.vechicle_static_param = initializeStaticParam();
    vehicle_param.vechicle_dynamic_param = initializeDynamicParam();
    
    // 初始化车辆状态
    VehicleState vehicle_state = initializeVehicleState();
    
    // 初始化系统
    if (!Initialize(vehicle_param, can_channel)) {
        std::cerr << "错误：系统初始化失败！" << std::endl;
        return -1;
    }
    std::cout << "√ 系统初始化成功" << std::endl;
    
    // 主控制循环
    const int control_freq = 100;  // 控制频率100Hz
    const std::chrono::milliseconds loop_duration(1000 / control_freq);
    
    std::cout << "\n开始车辆控制循环，按Ctrl+C退出..." << std::endl;
    printSeparator();
    
    while (gRunning) {
        auto loop_start = std::chrono::steady_clock::now();
        
        // 更新车辆状态
        if (!UpdateVehicleState(vehicle_state)) {
            std::cerr << "警告：车辆状态更新失败" << std::endl;
            continue;
        }
        
        // 更新动态参数
        UpdateDynamicParam(vehicle_param.vechicle_dynamic_param);
        
        
        
        // 发送CAN控制消息
        if (!SendCanMessage(can_channel, 1)) {
            std::cerr << "警告：CAN消息发送失败" << std::endl;
            continue;
        }
        
        // 打印当前状态（每秒一次）
        static int print_counter = 0;
        if (++print_counter >= control_freq) {
            printVehicleState(vehicle_state);
            print_counter = 0;
        }
        
        // 控制循环定时
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        if (elapsed < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed);
        }

        break;
    }
    
    // // 程序退出前执行紧急停止
    // if (!EmergencyStop()) {
    //     std::cerr << "警告：紧急停止执行失败！" << std::endl;
    // }
    
    std::cout << "\n程序已安全退出" << std::endl;
    return 0;
} 