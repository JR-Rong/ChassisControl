/**
 * @file torque_allocator.cpp
 * @brief 扭矩分配器类实现
 */

#include "vehicle_control/torque_allocator.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

namespace chassis_control {
namespace vehicle_control {

TorqueAllocator::TorqueAllocator(const data_input::VehicleConfig& config) 
    : allocateMethod(AllocateMethod::EVEN),
      targetVehicleSpeed(0.0f),
      steeringAngle(0.0f) {
    
    // 初始化基本参数
    initialize(config.getMass(), config.getWheelRadius());
    
    // 获取车轮位置
    wheelPositions = config.getWheelPositions();
}

TorqueAllocator::TorqueAllocator(float vehicle_mass, float wheel_radius, 
                               const std::vector<std::pair<float, float>>& wheel_positions)
    : wheelPositions(wheel_positions),
      allocateMethod(AllocateMethod::EVEN),
      targetVehicleSpeed(0.0f),
      steeringAngle(0.0f) {
    
    // 初始化基本参数
    initialize(vehicle_mass, wheel_radius);
}

TorqueAllocator::~TorqueAllocator() {
    // 析构函数
}

void TorqueAllocator::initialize(float mass, float radius) {
    if (mass <= 0.0f) {
        std::cerr << "警告: 车辆质量应大于0，使用默认值1500kg" << std::endl;
        vehicleMass = 1500.0f;
    } else {
        vehicleMass = mass;
    }
    
    if (radius <= 0.0f) {
        std::cerr << "警告: 车轮半径应大于0，使用默认值0.35m" << std::endl;
        wheelRadius = 0.35f;
    } else {
        wheelRadius = radius;
    }
    
    // 初始化目标车轮速度数组
    size_t wheelCount = wheelPositions.size();
    if (wheelCount == 0) {
        // 如果没有车轮位置，默认为四轮车
        wheelCount = 4;
        wheelPositions = {
            {1.4f, 0.8f},   // 左前
            {1.4f, -0.8f},  // 右前
            {-1.4f, 0.8f},  // 左后
            {-1.4f, -0.8f}  // 右后
        };
    }
    
    // 初始化目标车轮速度(默认全为0)
    targetWheelSpeeds.resize(wheelCount, 0.0f);
}

void TorqueAllocator::setVehicleSpeed(float target_vehicle_speed) {
    targetVehicleSpeed = target_vehicle_speed;
    
    // 同时更新所有车轮的目标速度为相同值
    std::fill(targetWheelSpeeds.begin(), targetWheelSpeeds.end(), target_vehicle_speed);
}

void TorqueAllocator::setWheelSpeeds(const std::vector<float>& target_wheel_speeds) {
    if (target_wheel_speeds.size() != targetWheelSpeeds.size()) {
        std::cerr << "警告: 目标车轮速度数组大小不匹配" << std::endl;
        return;
    }
    
    targetWheelSpeeds = target_wheel_speeds;
    
    // 更新车辆目标速度为车轮速度的平均值
    targetVehicleSpeed = std::accumulate(target_wheel_speeds.begin(), 
                                         target_wheel_speeds.end(), 0.0f) 
                        / target_wheel_speeds.size();
}

void TorqueAllocator::setSteeringAngle(float steering_angle) {
    steeringAngle = steering_angle;
}

void TorqueAllocator::setAllocateMethod(AllocateMethod method) {
    allocateMethod = method;
}

void TorqueAllocator::setParameters(const AllocationParameters& params) {
    parameters = params;
}

std::vector<float> TorqueAllocator::allocate(const std::vector<WheelState>& wheel_states, float accel) {
    // 检查输入有效性
    if (wheel_states.empty()) {
        std::cerr << "错误: 无效的车轮状态数组" << std::endl;
        return {};
    }
    
    // 计算需要分配的总扭矩
    float totalTorque = vehicleMass * accel * wheelRadius;
    
    // 根据分配方法分配扭矩
    std::vector<float> allocatedTorques;
    
    switch (allocateMethod) {
        case AllocateMethod::PROPORTIONAL:
            allocatedTorques = proportionalAllocate(wheel_states, totalTorque);
            break;
        
        case AllocateMethod::OPTIMAL:
            allocatedTorques = optimizeTorque(wheel_states, totalTorque);
            break;
        
        case AllocateMethod::YAW_CONTROL:
            allocatedTorques = yawControlAllocate(wheel_states, totalTorque, parameters.yawMomentTarget);
            break;
            
        case AllocateMethod::DYNAMIC_WEIGHT: {
            // 基于车轮垂直载荷的动态权重分配
            std::vector<float> weights(wheel_states.size());
            for (size_t i = 0; i < wheel_states.size(); ++i) {
                weights[i] = wheel_states[i].wheelModel.getNormalForce();
            }
            
            // 归一化权重
            float totalWeight = std::accumulate(weights.begin(), weights.end(), 0.0f);
            if (totalWeight > 0.001f) {
                for (auto& w : weights) {
                    w /= totalWeight;
                }
                
                // 按权重分配
                allocatedTorques.resize(wheel_states.size(), 0.0f);
                for (size_t i = 0; i < wheel_states.size(); ++i) {
                    allocatedTorques[i] = totalTorque * weights[i];
                }
            } else {
                // 如果总权重接近零，则均匀分配
                allocatedTorques = evenAllocate(totalTorque, wheel_states.size());
            }
            break;
        }
        
        case AllocateMethod::EVEN:
        default:
            allocatedTorques = evenAllocate(totalTorque, wheel_states.size());
            break;
    }
    
    // 应用扭矩约束
    return applyTorqueConstraints(allocatedTorques, parameters.maxTorquePerWheel);
}

std::vector<float> TorqueAllocator::optimizeTorque(const std::vector<WheelState>& wheel_states, float total_torque) {
    // 注意：实际的优化算法(如二次规划)通常需要专用的优化库
    // 这里提供一个简化版本，它考虑了轮胎滑移和横摆力矩
    
    size_t wheelCount = wheel_states.size();
    
    // 简化版本：结合均匀分配和横摆控制
    std::vector<float> baseTorques = evenAllocate(total_torque, wheelCount);
    std::vector<float> yawTorques = yawControlAllocate(wheel_states, 0.0f, parameters.yawMomentTarget);
    
    // 组合两种分配结果
    std::vector<float> optimizedTorques(wheelCount);
    for (size_t i = 0; i < wheelCount; ++i) {
        // 权重组合
        optimizedTorques[i] = baseTorques[i] * parameters.weightLongitudinal + 
                              yawTorques[i] * parameters.weightYaw;
    }
    
    // 调整总扭矩符合原始需求
    float actualTotalTorque = std::accumulate(optimizedTorques.begin(), optimizedTorques.end(), 0.0f);
    if (std::abs(actualTotalTorque) > 0.001f) {
        float scaleFactor = total_torque / actualTotalTorque;
        for (auto& torque : optimizedTorques) {
            torque *= scaleFactor;
        }
    }
    
    return optimizedTorques;
}

AllocateMethod TorqueAllocator::getAllocateMethod() const {
    return allocateMethod;
}

AllocationParameters TorqueAllocator::getParameters() const {
    return parameters;
}

std::vector<float> TorqueAllocator::evenAllocate(float total_torque, size_t wheel_count) const {
    if (wheel_count == 0) {
        return {};
    }
    
    // 均匀分配
    float torquePerWheel = total_torque / static_cast<float>(wheel_count);
    return std::vector<float>(wheel_count, torquePerWheel);
}

std::vector<float> TorqueAllocator::proportionalAllocate(
    const std::vector<WheelState>& wheel_states, 
    float total_torque) const {
    
    size_t wheelCount = wheel_states.size();
    if (wheelCount == 0) {
        return {};
    }
    
    // 基于滑移率的权重
    std::vector<float> weights(wheelCount);
    
    // 计算每个车轮的权重(基于滑移率倒数)
    // 滑移率越小，表示轮胎抓地力越好，分配的扭矩越多
    float totalWeight = 0.0f;
    
    for (size_t i = 0; i < wheelCount; ++i) {
        float slipRatio = std::abs(wheel_states[i].slipRatio);
        
        // 滑移率在安全范围内时，权重随滑移率减小而增加
        // 简单公式: weight = exp(-10 * slip^2)
        weights[i] = std::exp(-10.0f * slipRatio * slipRatio);
        
        // 累计总权重
        totalWeight += weights[i];
    }
    
    // 归一化权重并分配扭矩
    std::vector<float> allocatedTorques(wheelCount, 0.0f);
    
    if (totalWeight > 0.001f) {
        for (size_t i = 0; i < wheelCount; ++i) {
            // 归一化权重
            weights[i] /= totalWeight;
            
            // 按权重分配扭矩
            allocatedTorques[i] = total_torque * weights[i];
        }
    } else {
        // 如果总权重接近零(可能所有车轮都严重打滑)，则均匀分配
        float torquePerWheel = total_torque / static_cast<float>(wheelCount);
        std::fill(allocatedTorques.begin(), allocatedTorques.end(), torquePerWheel);
    }
    
    return allocatedTorques;
}

std::vector<float> TorqueAllocator::yawControlAllocate(
    const std::vector<WheelState>& wheel_states, 
    float total_torque, 
    float yaw_moment_target) const {
    
    size_t wheelCount = wheel_states.size();
    if (wheelCount == 0) {
        return {};
    }
    
    // 首先进行均匀分配
    std::vector<float> baseTorques = evenAllocate(total_torque, wheelCount);
    
    // 如果目标横摆力矩接近零，直接返回均匀分配
    if (std::abs(yaw_moment_target) < 0.001f) {
        return baseTorques;
    }
    
    // 计算每个车轮产生的横摆力矩系数(基于其位置)
    std::vector<float> yawFactors(wheelCount);
    
    for (size_t i = 0; i < wheelCount; ++i) {
        // 根据车轮位置计算横摆力矩系数
        // 横摆力矩 = 力 * 力臂
        // 力臂是车轮到车辆中心的垂直距离(y坐标)
        if (i < wheelPositions.size()) {
            // 使用车轮的y坐标作为力臂
            yawFactors[i] = wheelPositions[i].second;
        } else {
            // 若车轮位置信息不足，使用默认布局
            // 假设车辆坐标系：前方为x正方向，左侧为y正方向
            // 左侧车轮产生正的横摆力矩，右侧车轮产生负的横摆力矩
            bool isLeftWheel = (i % 2 == 0);
            yawFactors[i] = isLeftWheel ? 1.0f : -1.0f;
        }
    }
    
    // 计算当前分配下的总横摆力矩
    float currentYawMoment = 0.0f;
    for (size_t i = 0; i < wheelCount; ++i) {
        currentYawMoment += baseTorques[i] * yawFactors[i];
    }
    
    // 计算需要增加的横摆力矩
    float yawMomentDelta = yaw_moment_target - currentYawMoment;
    
    // 如果已经接近目标，不需要调整
    if (std::abs(yawMomentDelta) < 0.001f) {
        return baseTorques;
    }
    
    // 分离左右侧车轮(基于横摆力矩系数)
    std::vector<size_t> leftWheelIndices, rightWheelIndices;
    
    for (size_t i = 0; i < wheelCount; ++i) {
        if (yawFactors[i] > 0.0f) {
            leftWheelIndices.push_back(i);
        } else if (yawFactors[i] < 0.0f) {
            rightWheelIndices.push_back(i);
        }
        // 忽略横摆系数为零的车轮
    }
    
    // 根据需要增加或减少的横摆力矩调整扭矩分配
    std::vector<float> adjustedTorques = baseTorques;
    
    if (yawMomentDelta > 0.0f) {
        // 需要增加正向横摆力矩(向左偏转)
        // 增加左侧车轮扭矩，减少右侧车轮扭矩
        float adjustFactor = 0.5f * yawMomentDelta / (leftWheelIndices.size() + rightWheelIndices.size());
        
        for (size_t idx : leftWheelIndices) {
            adjustedTorques[idx] += adjustFactor;
        }
        
        for (size_t idx : rightWheelIndices) {
            adjustedTorques[idx] -= adjustFactor;
        }
    } else {
        // 需要增加负向横摆力矩(向右偏转)
        // 减少左侧车轮扭矩，增加右侧车轮扭矩
        float adjustFactor = 0.5f * std::abs(yawMomentDelta) / (leftWheelIndices.size() + rightWheelIndices.size());
        
        for (size_t idx : leftWheelIndices) {
            adjustedTorques[idx] -= adjustFactor;
        }
        
        for (size_t idx : rightWheelIndices) {
            adjustedTorques[idx] += adjustFactor;
        }
    }
    
    // 调整总扭矩符合原始需求
    float adjustedTotalTorque = std::accumulate(adjustedTorques.begin(), adjustedTorques.end(), 0.0f);
    if (std::abs(adjustedTotalTorque - total_torque) > 0.001f && std::abs(adjustedTotalTorque) > 0.001f) {
        float scaleFactor = total_torque / adjustedTotalTorque;
        for (auto& torque : adjustedTorques) {
            torque *= scaleFactor;
        }
    }
    
    return adjustedTorques;
}

std::vector<float> TorqueAllocator::applyTorqueConstraints(
    const std::vector<float>& torques, 
    float max_torque) const {
    
    std::vector<float> constrainedTorques = torques;
    
    // 应用最大扭矩限制
    for (auto& torque : constrainedTorques) {
        torque = std::min(std::max(torque, -max_torque), max_torque);
    }
    
    return constrainedTorques;
}

} // namespace vehicle_control
} // namespace chassis_control 