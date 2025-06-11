#include "can_bus_interface.h"
#include "global_variables.h"
#include <cstring>
#include <iostream>
#include <iomanip>

bool SendCanMessage(const std::string& channel_name,int can_id) {
    std::cout << "\n=== 全局变量状态 ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    
    // 打印静态参数
    const auto& static_param = GetStaticParam();
    std::cout << "\n【静态参数】" << std::endl;
    std::cout << "最大转向角: " << static_param.max_steering_angle << " rad" << std::endl;
    std::cout << "最大扭矩: " << static_param.max_torque << " Nm" << std::endl;
    std::cout << "最大速度: " << static_param.max_velocity << " m/s" << std::endl;
    std::cout << "最大加速度: " << static_param.max_acceleration << " m/s²" << std::endl;
    std::cout << "最大减速度: " << static_param.max_deceleration << " m/s²" << std::endl;
    std::cout << "车轮数量: " << static_param.wheel_count << std::endl;
    
    // 打印动态参数
    const auto& dynamic_param = GetDynamicParam();
    std::cout << "\n【动态参数】" << std::endl;
    std::cout << "质量: " << dynamic_param.mass << " kg" << std::endl;
    std::cout << "质心位置: (" << dynamic_param.cog_position[0] << "," 
              << dynamic_param.cog_position[1] << "," 
              << dynamic_param.cog_position[2] << ") m" << std::endl;

    for(int i=0;i<dynamic_param.wheel_params.size();i++){
        const auto& wheel = dynamic_param.wheel_params[i];
        std::cout << "车轮 " << i+1 << " 参数:" << std::endl;
        std::cout << "  轮胎行列位置: " << wheel.wheel_row << "," << wheel.wheel_col << std::endl;
        std::cout << "  特征名称: " << wheel.feture_name << std::endl;
        std::cout << "  摩擦系数: " << wheel.mu << std::endl;
        std::cout << "  滑移阈值: " << wheel.slip_threshold << std::endl;
        std::cout << "  轮胎宽度: " << wheel.wheel_width << " m" << std::endl;
        std::cout << "  轮胎半径: " << wheel.wheel_radius << " m" << std::endl;
        std::cout << "  轮胎位置: (" 
                  << wheel.wheel_position[0] << "," 
                  << wheel.wheel_position[1] << "," 
                  << wheel.wheel_position[2] 
                  << ") m" << std::endl;
        std::cout << "  胎压: " << wheel.wheel_pressure << " kPa" << std::endl;
    }    
    
    // 打印车辆状态
    const auto& vehicle_state = GetVehicleState();
    std::cout << "\n【车辆状态】" << std::endl;
    std::cout << "当前速度: (" << vehicle_state.cur_velocity[0] << "," 
              << vehicle_state.cur_velocity[1] << ") m/s" << std::endl;
    std::cout << "当前加速度: (" << vehicle_state.cur_acceleration[0] << "," 
              << vehicle_state.cur_acceleration[1] << ") m/s²" << std::endl;
    std::cout << "当前转向角: " << vehicle_state.cur_steering_angle << " rad" << std::endl;
    std::cout << "目标速度: (" << vehicle_state.target_velocity[0] << "," 
              << vehicle_state.target_velocity[1] << ") m/s" << std::endl;
    std::cout << "目标加速度: (" << vehicle_state.target_acceleration[0] << "," 
              << vehicle_state.target_acceleration[1] << ") m/s²" << std::endl;
    std::cout << "目标转向角: " << vehicle_state.target_steering_angle << " rad" << std::endl;
    std::cout << "目标位置: (" << vehicle_state.target_position[0] << "," 
              << vehicle_state.target_position[1] << "," 
              << vehicle_state.target_position[2] << ") m" << std::endl;
    std::cout << "车辆姿态: (" << vehicle_state.vehicle_posture[0] << "," 
              << vehicle_state.vehicle_posture[1] << "," 
              << vehicle_state.vehicle_posture[2] << ")" << std::endl;
    std::cout << "车辆位置: (" << vehicle_state.vehicle_position[0] << "," 
              << vehicle_state.vehicle_position[1] << "," 
              << vehicle_state.vehicle_position[2] << ") m" << std::endl;

    // 打印车轮状态
    for(int i=0;i<vehicle_state.tier_state.size();i++){
        const auto& wheel_state = vehicle_state.tier_state[i];
        std::cout << "车轮 " << i+1 << " 状态:" << std::endl;
        std::cout << "  轮胎行列位置: " << wheel_state.wheel_row << "," << wheel_state.wheel_col << std::endl;
        std::cout << "  转向角: (" << wheel_state.steeringAngle[0] << "," 
                    << wheel_state.steeringAngle[1] << ") rad" << std::endl;
        std::cout << "  角速度: (" << wheel_state.angularVelocity[0] << "," 
                    << wheel_state.angularVelocity[1] << ") rad/s" << std::endl;
        std::cout << "  是否滑移: " << (wheel_state.is_sliding ? "是" : "否") << std::endl;
        std::cout << "  垂向位移: " << wheel_state.vertical_displacement << " m" << std::endl;          
    }    
    std::cout << "\n=== 全局变量状态打印完成 ===" << std::endl;

    // 打印车轮控制信息
    const auto& control_info = GetControlInfo();
    std::cout << "\n【车轮控制信息】" << std::endl;
    for(int i=0; i<control_info.size(); i++) {
        const auto& wheel_control = control_info[i];
        std::cout << "车轮 " << i+1 << " 控制信息:" << std::endl;
        std::cout << "  轮胎行列位置: " << wheel_control.wheel_row << "," << wheel_control.wheel_col << std::endl;
        std::cout << "  车轮转速: (" << wheel_control.wheel_speed[0] << "," 
                  << wheel_control.wheel_speed[1] << ") rad/s" << std::endl;
        std::cout << "  车轮扭矩: (" << wheel_control.wheel_torque[0] << "," 
                  << wheel_control.wheel_torque[1] << ") Nm" << std::endl;
        std::cout << "  车轮转角: (" << wheel_control.wheel_angle[0] << "," 
                  << wheel_control.wheel_angle[1] << ") rad" << std::endl;
        std::cout << "  制动力: " << wheel_control.breaking_force << " N" << std::endl;
    }

    std::cout << "\n=== 车轮控制信息打印完成 ===" << std::endl;
    
    return true;
} 