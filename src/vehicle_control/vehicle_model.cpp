/**
 * @file vehicle_model.cpp
 * @brief 整车运动学模型类实现
 */

#include "vehicle_control/vehicle_model.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace chassis_control {
namespace vehicle_control {

VehicleModel::VehicleModel(std::shared_ptr<data_input::VehicleConfig> vehicle_config)
    : vehicleConfig_(vehicle_config),
      driveMode_(DriveMode::NORMAL),
      wheelCount_(4),  // 默认为四轮车
      wheelBase_(2.8f),
      trackWidth_(1.6f) {
    
    // 初始化驱动模式系数
    normalModeFactors_ = {1.0f, 1.0f, 1.0f, 1.0f};            // 均匀分配
    sportModeFactors_ = {0.8f, 0.8f, 1.2f, 1.2f};             // 后轮偏重
    ecoModeFactors_ = {1.2f, 1.2f, 0.8f, 0.8f};               // 前轮偏重(假设前轮驱动更省能)
    offRoadModeFactors_ = {1.1f, 1.1f, 1.1f, 1.1f};           // 全轮加强
    
    // 获取车辆配置
    if (vehicleConfig_) {
        maxSteeringAngle_ = vehicleConfig_->getMaxSteeringAngle();
        wheelRadius_ = vehicleConfig_->getWheelRadius();
        wheelBase_ = vehicleConfig_->getWheelBase();
        trackWidth_ = vehicleConfig_->getTrackWidth();
        
        // 初始化车轮状态，每个车轮使用配置的半径
        wheelStates_.resize(wheelCount_, WheelState(wheelRadius_));
        
        // 初始化车轮位置
        initializeDefaultWheelPositions();
    }
    else {
        // 默认值
        maxSteeringAngle_ = 0.6f;  // 约35度
        wheelRadius_ = 0.35f;      // 35cm
        
        // 初始化车轮状态
        wheelStates_.resize(wheelCount_, WheelState(wheelRadius_));
        
        // 初始化车轮位置
        initializeDefaultWheelPositions();
        
        std::cerr << "警告: 未提供车辆配置，使用默认值" << std::endl;
    }
    
    // 重置模型
    reset();
}

VehicleModel::VehicleModel(const std::vector<std::pair<float, float>>& wheel_positions)
    : vehicleConfig_(nullptr),
      wheelPositions_(wheel_positions),
      driveMode_(DriveMode::NORMAL),
      wheelCount_(wheel_positions.size()),
      wheelBase_(2.8f),
      trackWidth_(1.6f),
      maxSteeringAngle_(0.6f),
      wheelRadius_(0.35f) {
    
    // 初始化驱动模式系数
    normalModeFactors_ = {1.0f, 1.0f, 1.0f, 1.0f};
    sportModeFactors_ = {0.8f, 0.8f, 1.2f, 1.2f};
    ecoModeFactors_ = {1.2f, 1.2f, 0.8f, 0.8f};
    offRoadModeFactors_ = {1.1f, 1.1f, 1.1f, 1.1f};
    
    // 初始化车轮状态
    wheelStates_.resize(wheelCount_, WheelState(wheelRadius_));
    
    // 计算轴距和轮距（基于提供的车轮位置）
    if (wheelCount_ >= 2) {
        // 查找最大x和y值
        float maxX = 0.0f, maxY = 0.0f;
        for (const auto& pos : wheelPositions_) {
            maxX = std::max(maxX, std::abs(pos.first));
            maxY = std::max(maxY, std::abs(pos.second));
        }
        
        // 估算轴距和轮距
        wheelBase_ = maxX * 2.0f;
        trackWidth_ = maxY * 2.0f;
    }
    
    // 重置模型
    reset();
}

VehicleModel::~VehicleModel() {
    // 析构函数
}

bool VehicleModel::update(const ControlCommand& command, 
                          std::shared_ptr<data_input::StateEstimator> state_estimator) {
    if (!state_estimator) {
        std::cerr << "错误: 无效的状态估计器" << std::endl;
        return false;
    }
    
    // 获取车辆状态
    float currentSpeed = state_estimator->getFilteredSpeed();
    float currentAccel = state_estimator->getFilteredAcceleration();
    std::vector<float> attitude = state_estimator->getEstimatedAttitude();
    
    // 确保姿态数据有效
    if (attitude.size() < 6) {
        std::cerr << "错误: 无效的姿态数据" << std::endl;
        return false;
    }
    
    // 获取目标状态
    float targetSpeed = command.targetSpeed;
    float targetAccel = command.targetAcceleration;
    float targetYawRate = command.targetYawRate;
    float steeringAngle = command.steeringAngle;
    
    // 限制转向角
    steeringAngle = std::min(std::max(steeringAngle, -maxSteeringAngle_), maxSteeringAngle_);
    
    // 使用默认阿克曼转向模型计算车轮状态
    SteeringFunction ackermanFn = getAckermanSteeringFunction();
    std::vector<WheelState> states = computeWheelStates(currentSpeed, targetAccel, steeringAngle, ackermanFn);
    
    // 更新车轮状态
    if (states.size() == wheelStates_.size()) {
        wheelStates_ = states;
        return true;
    }
    else {
        std::cerr << "错误: 计算的车轮状态数量与模型不符" << std::endl;
        return false;
    }
}

std::vector<WheelState> VehicleModel::computeWheelStates(
    float v, 
    float a, 
    float steer,
    SteeringFunction steering_model) {
    
    std::vector<WheelState> states = wheelStates_;
    
    // 限制转向角
    steer = std::min(std::max(steer, -maxSteeringAngle_), maxSteeringAngle_);
    
    // 估算车辆最大垂直载荷（假设）
    float totalMass = vehicleConfig_ ? vehicleConfig_->getMass() : 1500.0f;
    float gravity = 9.81f;
    float totalWeight = totalMass * gravity;
    
    // 简单垂直载荷分配（可以根据动态荷重转移更精确计算）
    float loadPerWheel = totalWeight / wheelCount_;
    
    // 估算横摆角速度 (简化模型)
    float yawRate = v > 0.1f ? (v * std::tan(steer) / wheelBase_) : 0.0f;
    
    // 计算各轮状态
    for (size_t i = 0; i < wheelCount_; ++i) {
        // 应用转向模型计算各轮转向角
        states[i].steeringAngle = steering_model(i, steer);
        
        // 计算车轮在其自身坐标系中的线速度
        float wheelX = wheelPositions_[i].first;
        float wheelY = wheelPositions_[i].second;
        
        // 计算车轮中心点的速度
        float vx = v;  // 车辆前进方向速度
        float vy = 0.0f;  // 车辆横向速度（简化模型中忽略侧滑）
        
        // 考虑旋转导致的额外速度: v = ω × r
        float rotVx = -yawRate * wheelY;
        float rotVy = yawRate * wheelX;
        
        // 车轮中心总速度
        float wheelVx = vx + rotVx;
        float wheelVy = vy + rotVy;
        
        // 计算车轮在转向后的坐标系中的速度
        float cosSteer = std::cos(states[i].steeringAngle);
        float sinSteer = std::sin(states[i].steeringAngle);
        float wheelDirectionalV = wheelVx * cosSteer + wheelVy * sinSteer;
        
        // 设置车轮期望线速度
        states[i].desiredSpeed = wheelDirectionalV;
        
        // 设置车轮角速度 ω = v / r
        states[i].angularVelocity = wheelDirectionalV / wheelRadius_;
        
        // 更新轮胎模型
        states[i].wheelModel.updateLoad(loadPerWheel);
        
        // 计算滑移率
        float slipRatio = 0.0f;
        if (std::abs(wheelDirectionalV) > 0.001f) {
            float wheelCircumferentialV = states[i].angularVelocity * wheelRadius_;
            slipRatio = (wheelCircumferentialV - wheelDirectionalV) / std::abs(wheelDirectionalV);
        }
        states[i].slipRatio = slipRatio;
        states[i].wheelModel.updateSlipRatio(slipRatio);
        
        // 计算目标扭矩 (简化估算)
        float vehicleMass = vehicleConfig_ ? vehicleConfig_->getMass() : 1500.0f;
        float totalTorque = vehicleMass * a * wheelRadius_;
        float torquePerWheel = totalTorque / wheelCount_;
        
        // 应用驱动模式系数
        std::array<float, 4> wheelTorques = {
            torquePerWheel, torquePerWheel, torquePerWheel, torquePerWheel
        };
        
        if (i < 4) {  // 只对前四个轮子应用驱动模式系数
            wheelTorques = applyDriveModeFactors(wheelTorques);
            states[i].targetTorque = i < wheelTorques.size() ? wheelTorques[i] : torquePerWheel;
        }
        else {
            states[i].targetTorque = torquePerWheel;
        }
        
        // 更新轮胎模型的输入扭矩
        states[i].wheelModel.updateTorqueInput(states[i].targetTorque);
        
        // 方向角等于转向角(简化)
        states[i].directionAngle = states[i].steeringAngle;
    }
    
    return states;
}

WheelState VehicleModel::getWheelState(size_t wheel_index) const {
    if (wheel_index < wheelStates_.size()) {
        return wheelStates_[wheel_index];
    }
    else {
        std::cerr << "错误: 无效的车轮索引" << wheel_index << std::endl;
        return WheelState(wheelRadius_);
    }
}

std::vector<WheelState> VehicleModel::getAllWheelStates() const {
    return wheelStates_;
}

void VehicleModel::setDriveMode(DriveMode mode) {
    driveMode_ = mode;
}

DriveMode VehicleModel::getDriveMode() const {
    return driveMode_;
}

void VehicleModel::reset() {
    // 重置所有车轮状态
    for (auto& state : wheelStates_) {
        state = WheelState(wheelRadius_);
    }
}

size_t VehicleModel::getWheelCount() const {
    return wheelCount_;
}

void VehicleModel::setWheelBase(float wheelbase) {
    if (wheelbase > 0.0f) {
        wheelBase_ = wheelbase;
    }
    else {
        std::cerr << "警告: 轴距应大于0" << std::endl;
    }
}

void VehicleModel::setTrackWidth(float trackwidth) {
    if (trackwidth > 0.0f) {
        trackWidth_ = trackwidth;
    }
    else {
        std::cerr << "警告: 轮距应大于0" << std::endl;
    }
}

float VehicleModel::getWheelBase() const {
    return wheelBase_;
}

float VehicleModel::getTrackWidth() const {
    return trackWidth_;
}

SteeringFunction VehicleModel::getAckermanSteeringFunction() {
    // 返回默认阿克曼转向函数
    return [this](int wheelIndex, float steerInput) -> float {
        // 检查是否为有效的车轮索引
        if (wheelIndex < 0 || wheelIndex >= static_cast<int>(wheelCount_)) {
            return 0.0f;
        }
        
        // 如果转向角几乎为0，直接返回0
        if (std::abs(steerInput) < 1e-5) {
            return 0.0f;
        }
        
        // 只有前轮转向
        if (wheelIndex >= 2) {
            return 0.0f;
        }
        
        // 计算转向半径
        float steeringRadius = wheelBase_ / std::tan(std::abs(steerInput));
        
        // 左前轮(0)和右前轮(1)
        if (steerInput > 0) { // 左转
            if (wheelIndex == 0) { // 左前轮（内侧）
                return std::atan(wheelBase_ / (steeringRadius - trackWidth_/2));
            } else { // 右前轮（外侧）
                return std::atan(wheelBase_ / (steeringRadius + trackWidth_/2));
            }
        } else { // 右转
            if (wheelIndex == 0) { // 左前轮（外侧）
                return -std::atan(wheelBase_ / (steeringRadius + trackWidth_/2));
            } else { // 右前轮（内侧）
                return -std::atan(wheelBase_ / (steeringRadius - trackWidth_/2));
            }
        }
    };
}

std::array<float, 4> VehicleModel::calculateAckermanSteering(float steering_angle) const {
    // 获取车辆尺寸
    float wheelBase = wheelBase_;
    float trackWidth = trackWidth_;
    
    // 阿克曼转向几何计算
    // 内外轮转角不同，内轮角度更大
    std::array<float, 4> wheelAngles = {0.0f, 0.0f, 0.0f, 0.0f}; // [左前，右前，左后，右后]
    
    if (std::abs(steering_angle) < 1e-5) {
        // 直行，所有轮子角度为0
        return wheelAngles;
    }
    
    // 计算转向半径
    float steeringRadius = wheelBase / std::tan(std::abs(steering_angle));
    
    // 符号：左转为正，右转为负
    int sign = (steering_angle > 0) ? 1 : -1;
    
    if (steering_angle > 0) { // 左转
        // 前轮转角
        wheelAngles[0] = std::atan(wheelBase / (steeringRadius - trackWidth/2)); // 左前轮（内侧）
        wheelAngles[1] = std::atan(wheelBase / (steeringRadius + trackWidth/2)); // 右前轮（外侧）
        
        // 后轮通常不转向，但如果是四轮转向系统，可以添加代码
        wheelAngles[2] = 0.0f; // 左后轮
        wheelAngles[3] = 0.0f; // 右后轮
    }
    else { // 右转
        // 前轮转角
        wheelAngles[0] = -std::atan(wheelBase / (steeringRadius + trackWidth/2)); // 左前轮（外侧）
        wheelAngles[1] = -std::atan(wheelBase / (steeringRadius - trackWidth/2)); // 右前轮（内侧）
        
        // 后轮通常不转向
        wheelAngles[2] = 0.0f; // 左后轮
        wheelAngles[3] = 0.0f; // 右后轮
    }
    
    return wheelAngles;
}

std::array<float, 4> VehicleModel::calculateWheelAngularVelocities(
    float linear_velocity, 
    float yaw_rate, 
    const std::array<float, 4>& wheel_steering_angles) const {
    
    // 获取车辆尺寸和车轮位置
    float wheelBase = wheelBase_;
    float trackWidth = trackWidth_;
    
    // 使用车轮位置
    std::array<std::pair<float, float>, 4> wheelCoords;
    
    // 如果车轮位置有效
    if (wheelPositions_.size() >= 4) {
        for (size_t i = 0; i < 4; ++i) {
            wheelCoords[i] = wheelPositions_[i];
        }
    }
    else {
        // 使用默认车轮位置
        wheelCoords[0] = {wheelBase/2, trackWidth/2};   // 左前
        wheelCoords[1] = {wheelBase/2, -trackWidth/2};  // 右前
        wheelCoords[2] = {-wheelBase/2, trackWidth/2};  // 左后
        wheelCoords[3] = {-wheelBase/2, -trackWidth/2}; // 右后
    }
    
    std::array<float, 4> wheelAngularVelocities;
    
    for (size_t i = 0; i < 4; ++i) {
        // 计算车轮实际线速度
        float wheelX = wheelCoords[i].first;
        float wheelY = wheelCoords[i].second;
        
        // 考虑偏航角速度导致的额外速度
        float vx = linear_velocity;
        float vy = 0.0f;
        
        // 旋转导致的速度分量 v = ω × r
        float rotationVx = -yaw_rate * wheelY;
        float rotationVy = yaw_rate * wheelX;
        
        // 车轮总速度
        float wheelVx = vx + rotationVx;
        float wheelVy = vy + rotationVy;
        
        // 车轮在其前进方向上的速度分量
        float wheelDirectionalV = wheelVx * std::cos(wheel_steering_angles[i]) + 
                                 wheelVy * std::sin(wheel_steering_angles[i]);
        
        // 计算角速度 ω = v / r
        wheelAngularVelocities[i] = wheelDirectionalV / wheelRadius_;
    }
    
    return wheelAngularVelocities;
}

std::array<float, 4> VehicleModel::calculateSlipRatios(
    const std::array<float, 4>& wheel_angular_velocities,
    float linear_velocity,
    const std::array<float, 4>& wheel_steering_angles) const {
    
    std::array<float, 4> slipRatios;
    
    for (size_t i = 0; i < 4; ++i) {
        // 车轮理论线速度
        float wheelTheoreticalSpeed = wheel_angular_velocities[i] * wheelRadius_;
        
        // 轮地接触点处的实际速度在车轮方向上的分量
        float wheelActualSpeed = linear_velocity * std::cos(wheel_steering_angles[i]);
        
        // 防止除零
        if (std::abs(wheelActualSpeed) < 1e-5) {
            slipRatios[i] = 0.0f;
        }
        else {
            // 滑移率计算: (理论速度 - 实际速度) / max(|理论速度|, |实际速度|)
            slipRatios[i] = (wheelTheoreticalSpeed - wheelActualSpeed) / 
                            std::max(std::abs(wheelTheoreticalSpeed), std::abs(wheelActualSpeed));
        }
    }
    
    return slipRatios;
}

std::array<float, 4> VehicleModel::applyDriveModeFactors(const std::array<float, 4>& base_values) const {
    std::array<float, 4> result;
    const std::array<float, 4>* factors = nullptr;
    
    // 根据驱动模式选择系数
    switch (driveMode_) {
        case DriveMode::SPORT:
            factors = &sportModeFactors_;
            break;
        case DriveMode::ECO:
            factors = &ecoModeFactors_;
            break;
        case DriveMode::OFF_ROAD:
            factors = &offRoadModeFactors_;
            break;
        case DriveMode::NORMAL:
        default:
            factors = &normalModeFactors_;
            break;
    }
    
    // 应用系数
    for (size_t i = 0; i < 4; ++i) {
        result[i] = base_values[i] * (*factors)[i];
    }
    
    return result;
}

void VehicleModel::initializeDefaultWheelPositions() {
    // 基于轴距和轮距计算默认的车轮位置
    // 坐标系：前方为x正方向，左侧为y正方向，上方为z正方向
    // 车辆中心为原点(0,0,0)
    
    // 如果已有位置，不重新初始化
    if (!wheelPositions_.empty() && wheelPositions_.size() == wheelCount_) {
        return;
    }
    
    // 清空现有位置
    wheelPositions_.clear();
    
    // 四轮车的标准位置
    if (wheelCount_ == 4) {
        // 左前
        wheelPositions_.push_back({wheelBase_/2, trackWidth_/2});
        // 右前
        wheelPositions_.push_back({wheelBase_/2, -trackWidth_/2});
        // 左后
        wheelPositions_.push_back({-wheelBase_/2, trackWidth_/2});
        // 右后
        wheelPositions_.push_back({-wheelBase_/2, -trackWidth_/2});
    }
    // 六轮车(假设三轴，每轴两轮)
    else if (wheelCount_ == 6) {
        float axleSpacing = wheelBase_ / 2;  // 轴间距
        
        // 左前
        wheelPositions_.push_back({wheelBase_/2, trackWidth_/2});
        // 右前
        wheelPositions_.push_back({wheelBase_/2, -trackWidth_/2});
        // 左中
        wheelPositions_.push_back({0.0f, trackWidth_/2});
        // 右中
        wheelPositions_.push_back({0.0f, -trackWidth_/2});
        // 左后
        wheelPositions_.push_back({-wheelBase_/2, trackWidth_/2});
        // 右后
        wheelPositions_.push_back({-wheelBase_/2, -trackWidth_/2});
    }
    // 其他车轮数量(均匀分布假设)
    else {
        // 每侧车轮数
        size_t wheelsPerSide = wheelCount_ / 2;
        
        // 如果车轮数为奇数，则一侧多一个
        if (wheelCount_ % 2 != 0) {
            wheelsPerSide = wheelCount_ / 2 + 1;
        }
        
        // 计算轴间距
        float axleSpacing = wheelBase_ / (wheelsPerSide - 1);
        
        // 生成位置
        for (size_t i = 0; i < wheelsPerSide; ++i) {
            // 左侧
            wheelPositions_.push_back({wheelBase_/2 - i*axleSpacing, trackWidth_/2});
            
            // 右侧(如果车轮总数为奇数且这是最后一个轴，则不添加右侧轮)
            if (i < wheelsPerSide - 1 || wheelCount_ % 2 == 0) {
                wheelPositions_.push_back({wheelBase_/2 - i*axleSpacing, -trackWidth_/2});
            }
        }
    }
}

} // namespace vehicle_control
} // namespace chassis_control 