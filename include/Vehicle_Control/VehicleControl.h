/**
 * @file VehicleControl.h
 * @brief 车轮控制数据的计算模式头问题
 * 
 * 本文件定义了车辆控制数据计算模块的接口。
 */
#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <string>
#include <vector>
#include <map>
#include <array>
#include "VehicleDataInput.h"

struct WheelControlInformation {
    // 车轮控制信息
    int wheel_col = -1;                     // 车轮列索引
    int wheel_row = -1;                     // 车轮行索引
    std::array<float,2> wheel_speed;        // 车轮转速
    std::array<float,2> wheel_torque;       // 车轮扭矩
    std::array<float,2> wheel_angle;         // 车轮转角
    float breaking_force;                   // 制动力
};

void CalculateWheelControlData(const VehicleState& vehicle_state, const VehicleParam& vehicle_param);

#endif // VEHICLE_CONTROL_H