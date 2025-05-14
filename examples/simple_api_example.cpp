/**
 * @file simple_api_example.cpp
 * @brief 简单接口使用示例程序
 * 
 * 本示例展示了如何使用底盘控制系统的高层API接口。
 * 只需包含chassis_control.h一个头文件即可使用所有功能。
 */

#include "chassis_control.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

// 全局变量，用于处理程序终止信号
volatile sig_atomic_t gRunning = 1;

// 信号处理函数
void signalHandler(int signal) {
    std::cout << "接收到终止信号 " << signal << std::endl;
    gRunning = 0;
}

int main(int argc, char** argv) {
    // 注册信号处理器
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 创建底盘控制器实例
    chassis_control::ChassisController controller;
    
    try {
        // 初始化控制器参数
        bool init_success = controller.initialize(
            1500.0f,     // 车辆质量 (kg)
            0.35f,       // 轮胎半径 (m)
            2.7f,        // 轴距 (m)
            1.8f,        // 轮距 (m)
            30.0f,       // 最大转向角 (度)
            500.0f,      // 最大扭矩 (N.m)
            "can0"       // CAN通道
        );
        
        if (!init_success) {
            std::cerr << "初始化底盘控制器失败！" << std::endl;
            return 1;
        }
        
        // 设置控制模式
        controller.setControlMode(chassis_control::ChassisController::ControlMode::TORQUE_CONTROL);
        controller.setAllocator(chassis_control::ChassisController::AllocatorType::DYNAMIC_ALLOCATOR);
        controller.setSmoother(chassis_control::ChassisController::SmootherType::QUADRATIC_SMOOTHER);
        
        // 设置轮胎基本参数
        controller.setTireFrictionCoefficient(0.8f);      // 干燥路面摩擦系数
        controller.setTireSlipThreshold(0.15f);           // 滑移阈值
        
        // 配置不同轮胎的物理参数（展示个性化配置）
        // 前轮参数（索引0和1）
        controller.setTireWidth(0.245f, 0);               // 前左轮宽度
        controller.setTireWidth(0.245f, 1);               // 前右轮宽度
        controller.setTireStiffness(280000.0f, 0);        // 前左轮刚度
        controller.setTireStiffness(280000.0f, 1);        // 前右轮刚度
        controller.setTireDamping(5500.0f, 0);            // 前左轮阻尼
        controller.setTireDamping(5500.0f, 1);            // 前右轮阻尼
        controller.setTireCorneringStiffness(90000.0f, 0); // 前左轮侧偏刚度
        controller.setTireCorneringStiffness(90000.0f, 1); // 前右轮侧偏刚度
        
        // 后轮参数（索引2和3）
        controller.setTireWidth(0.265f, 2);               // 后左轮宽度
        controller.setTireWidth(0.265f, 3);               // 后右轮宽度
        controller.setTireStiffness(300000.0f, 2);        // 后左轮刚度
        controller.setTireStiffness(300000.0f, 3);        // 后右轮刚度
        controller.setTireDamping(6000.0f, 2);            // 后左轮阻尼
        controller.setTireDamping(6000.0f, 3);            // 后右轮阻尼
        controller.setTireCorneringStiffness(85000.0f, 2); // 后左轮侧偏刚度
        controller.setTireCorneringStiffness(85000.0f, 3); // 后右轮侧偏刚度
        
        std::cout << "底盘控制器初始化成功，开始车辆控制..." << std::endl;
        std::cout << "轮胎配置信息：" << std::endl;
        
        // 输出轮胎配置信息
        for (int i = 0; i < 4; i++) {
            std::cout << "轮胎 " << i << ": "
                      << "半径=" << controller.getTireRadius(i) << "m, "
                      << "宽度=" << controller.getTireWidth(i) << "m" << std::endl;
        }
        
        int cycle_count = 0;
        float current_speed = 0.0f;
        float target_speed = 0.0f;
        float steering_angle = 0.0f;
        
        // 主控制循环
        while (gRunning) {
            cycle_count++;
            
            // 模拟场景：展示车辆在各种路面和驾驶条件下的行为
            if (cycle_count < 1000) {
                // 阶段1：在干燥路面上加速
                target_speed = std::min(15.0f, current_speed + 0.1f);
                steering_angle = 0.0f;
            } else if (cycle_count < 2000) {
                // 阶段2：保持速度
                target_speed = 15.0f;
                steering_angle = 0.0f;
            } else if (cycle_count < 3000) {
                // 阶段3：转向并进入湿滑路面 (减小摩擦系数)
                if (cycle_count == 2000) {
                    std::cout << "\n============== 进入湿滑路面 ==============\n" << std::endl;
                    controller.setTireFrictionCoefficient(0.4f);  // 湿滑路面
                }
                target_speed = 15.0f;
                steering_angle = 20.0f;
            } else if (cycle_count < 4000) {
                // 阶段4：急转弯（测试侧偏力）
                target_speed = 15.0f;
                steering_angle = -25.0f;
            } else if (cycle_count < 5000) {
                // 阶段5：进入冰面（低摩擦系数）
                if (cycle_count == 4000) {
                    std::cout << "\n============== 进入冰面 ==============\n" << std::endl;
                    controller.setTireFrictionCoefficient(0.2f);  // 冰面
                }
                target_speed = 15.0f;
                steering_angle = 15.0f;
            } else if (cycle_count < 6000) {
                // 阶段6：急刹车测试打滑情况
                target_speed = std::max(0.0f, current_speed - 0.3f);
                steering_angle = 0.0f;
            } else {
                // 测试完成，退出
                break;
            }
            
            // 简单的速度模型 (真实系统中这将由传感器提供)
            current_speed = current_speed * 0.98f + target_speed * 0.02f;
            
            // 设置车辆控制指令
            controller.setTargetSpeed(current_speed);
            controller.setTargetAcceleration((target_speed - current_speed) * 2.0f); // 简化的加速度计算
            controller.setSteeringAngle(steering_angle);
            
            // 更新控制器 (这将计算轮胎扭矩并通过CAN总线发送)
            controller.update();
            
            // 每100个周期输出一次状态信息
            if (cycle_count % 100 == 0) {
                auto wheel_torques = controller.getWheelTorques();
                std::cout << "\n周期: " << cycle_count 
                          << " | 速度: " << current_speed << " m/s"
                          << " | 加速度: " << controller.getTargetAcceleration() << " m/s²"
                          << " | 转向角: " << steering_angle << "°" << std::endl;
                
                std::cout << "轮胎扭矩 (N·m): ";
                for (size_t i = 0; i < wheel_torques.size(); ++i) {
                    std::cout << wheel_torques[i] << " ";
                }
                std::cout << std::endl;
                
                // 输出轮胎动态状态
                std::cout << "滑移状态: ";
                for (int i = 0; i < 4; ++i) {
                    std::cout << (controller.getTireSliding(i) ? "滑移 " : "抓地 ");
                }
                std::cout << std::endl;
                
                std::cout << "滑移比: ";
                for (int i = 0; i < 4; ++i) {
                    std::cout << controller.getTireSlipRatio(i) << " ";
                }
                std::cout << std::endl;
                
                std::cout << "纵向力 (N): ";
                for (int i = 0; i < 4; ++i) {
                    std::cout << controller.getTireLongitudinalForce(i) << " ";
                }
                std::cout << std::endl;
                
                std::cout << "侧向力 (N): ";
                for (int i = 0; i < 4; ++i) {
                    std::cout << controller.getTireLateralForce(i) << " ";
                }
                std::cout << std::endl;
                
                std::cout << "轮胎负载 (N): ";
                for (int i = 0; i < 4; ++i) {
                    std::cout << controller.getTireLoad(i) << " ";
                }
                std::cout << std::endl;
                
                std::cout << "接触面积 (cm²): ";
                for (int i = 0; i < 4; ++i) {
                    std::cout << controller.getTireContactPatchArea(i) * 10000.0f << " ";
                }
                std::cout << std::endl;
                
                std::cout << "垂直位移 (mm): ";
                for (int i = 0; i < 4; ++i) {
                    std::cout << controller.getTireVerticalDisplacement(i) * 1000.0f << " ";
                }
                std::cout << std::endl;
            }
            
            // 循环延时模拟10ms的控制周期
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // 车辆停止前，发送紧急停止命令
        std::cout << "\n发送紧急停止命令..." << std::endl;
        controller.emergencyStop();
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "程序正常退出" << std::endl;
    return 0;
} 