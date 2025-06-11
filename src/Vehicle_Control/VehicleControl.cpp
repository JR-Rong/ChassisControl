#include "VehicleControl.h"
#include "global_variables.h"
#include <iostream>

/**
 * @brief 计算车轮控制数据
 * @param vehicle_state 当前车辆状态
 * @param vehicle_param 整车参数
 * @return bool 是否成功加载
 */
void CalculateWheelControlData(const VehicleState& vehicle_state, const VehicleParam& vehicle_param){
    std::vector<WheelControlInformation> wheel_control_info;

    int wheel_count = vehicle_param.vechicle_static_param.wheel_count;
    wheel_control_info.resize(wheel_count);

    for (int i = 0; i < wheel_count; ++i) {
        WheelControlInformation info;
        info.wheel_col = vehicle_param.vechicle_dynamic_param.wheel_params[i].wheel_col;
        info.wheel_row = vehicle_param.vechicle_dynamic_param.wheel_params[i].wheel_row;
        
        // 计算车轮转速、扭矩和角度（示例计算，实际应根据车辆动力学模型）
        info.wheel_speed[0] = vehicle_state.cur_velocity[0] * (1 + 0.1f * i); // 示例
        info.wheel_speed[1] = vehicle_state.cur_velocity[1] * (1 + 0.1f * i); // 示例
        info.wheel_torque[0] = vehicle_param.vechicle_static_param.max_torque * (1 - 0.05f * i); // 示例
        info.wheel_torque[1] = vehicle_param.vechicle_static_param.max_torque * (1 - 0.05f * i); // 示例
        info.wheel_angle[0] = vehicle_state.cur_steering_angle * (1 + 0.05f * i); // 示例
        info.wheel_angle[1] = vehicle_state.cur_steering_angle * (1 + 0.05f * i); // 示例
        
        info.breaking_force = vehicle_param.vechicle_static_param.max_deceleration * (1 - 0.1f * i); // 示例
        wheel_control_info[i] = info;
    }

    SetControlInfo(wheel_control_info);
    std::cout << "CalculateWheelControlData" << std::endl;
    return;
}