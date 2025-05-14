/**
 * @file torque_smoother.cpp
 * @brief 扭矩平滑处理类实现
 */

#include "vehicle_control/torque_smoother.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <chrono>

namespace chassis_control {
namespace vehicle_control {

TorqueSmoother::TorqueSmoother()
    : smootherType_(SmootherType::LOW_PASS_FILTER),
      alpha_(0.1f),
      windowSize_(5),
      maxRateOfChange_(1000.0f),
      sampleTime_(0.01f) {
    
    // 默认为4个车轮（可扩展）
    const size_t wheelCount = 4;
    
    // 初始化历史数据窗口
    windows_.resize(wheelCount);
    lastOutputs_.resize(wheelCount, 0.0f);
    
    // 初始化PID相关数据
    pidParams_ = PIDParams();
    pidErrors_.resize(wheelCount, 0.0f);
    pidIntegrals_.resize(wheelCount, 0.0f);
    pidDerivatives_.resize(wheelCount, 0.0f);
    lastUpdateTimes_.resize(wheelCount, getCurrentTimestamp());
}

TorqueSmoother::TorqueSmoother(SmootherType smoother_type, float filter_alpha, size_t window_size)
    : smootherType_(smoother_type),
      alpha_(filter_alpha),
      windowSize_(window_size),
      maxRateOfChange_(1000.0f),
      sampleTime_(0.01f) {
    
    // 参数校验
    if (alpha_ < 0.0f || alpha_ > 1.0f) {
        alpha_ = 0.1f;
        std::cerr << "警告: 滤波系数应在0-1范围内，已重置为默认值0.1" << std::endl;
    }
    
    if (windowSize_ < 2) {
        windowSize_ = 5;
        std::cerr << "警告: 窗口大小应大于等于2，已重置为默认值5" << std::endl;
    }
    
    // 默认为4个车轮（可扩展）
    const size_t wheelCount = 4;
    
    // 初始化历史数据窗口
    windows_.resize(wheelCount);
    lastOutputs_.resize(wheelCount, 0.0f);
    
    // 初始化PID相关数据
    pidParams_ = PIDParams();
    pidErrors_.resize(wheelCount, 0.0f);
    pidIntegrals_.resize(wheelCount, 0.0f);
    pidDerivatives_.resize(wheelCount, 0.0f);
    lastUpdateTimes_.resize(wheelCount, getCurrentTimestamp());
}

TorqueSmoother::~TorqueSmoother() {
    // 析构函数
}

float TorqueSmoother::smoothTorque(float raw_torque, size_t wheel_index) {
    // 检查索引是否有效
    if (wheel_index >= windows_.size()) {
        std::cerr << "错误: 无效的车轮索引" << wheel_index << std::endl;
        return raw_torque;
    }
    
    float smoothed_torque = 0.0f;
    
    // 根据平滑器类型应用不同的平滑方法
    switch (smootherType_) {
        case SmootherType::LOW_PASS_FILTER:
            smoothed_torque = applyLowPassFilter(raw_torque, lastOutputs_[wheel_index]);
            break;
            
        case SmootherType::MOVING_AVERAGE:
            smoothed_torque = applyMovingAverage(raw_torque, windows_[wheel_index]);
            break;
            
        case SmootherType::PID_CONTROLLER:
            // 在PID模式下，raw_torque被视为目标值，lastOutputs_被视为当前值
            smoothed_torque = applyPIDControl(raw_torque, lastOutputs_[wheel_index], wheel_index);
            break;
            
        case SmootherType::RATE_LIMITER:
            smoothed_torque = applyRateLimiter(raw_torque, lastOutputs_[wheel_index]);
            break;
            
        default:
            smoothed_torque = raw_torque;
            break;
    }
    
    // 更新上次输出值
    lastOutputs_[wheel_index] = smoothed_torque;
    
    return smoothed_torque;
}

std::vector<float> TorqueSmoother::smoothTorques(const std::vector<float>& raw_torques) {
    std::vector<float> smoothed_torques;
    
    // 确保数据大小匹配
    if (raw_torques.size() > windows_.size()) {
        // 如果输入扭矩数量超过已初始化的车轮数量，扩展容器
        windows_.resize(raw_torques.size());
        lastOutputs_.resize(raw_torques.size(), 0.0f);
        pidErrors_.resize(raw_torques.size(), 0.0f);
        pidIntegrals_.resize(raw_torques.size(), 0.0f);
        pidDerivatives_.resize(raw_torques.size(), 0.0f);
        lastUpdateTimes_.resize(raw_torques.size(), getCurrentTimestamp());
    }
    
    // 处理每个扭矩值
    for (size_t i = 0; i < raw_torques.size(); ++i) {
        float smoothed = smoothTorque(raw_torques[i], i);
        smoothed_torques.push_back(smoothed);
    }
    
    return smoothed_torques;
}

void TorqueSmoother::setSmootherType(SmootherType type) {
    smootherType_ = type;
}

void TorqueSmoother::setFilterAlpha(float alpha) {
    if (alpha < 0.0f || alpha > 1.0f) {
        std::cerr << "警告: 滤波系数应在0-1范围内" << std::endl;
        return;
    }
    alpha_ = alpha;
}

void TorqueSmoother::setWindowSize(size_t size) {
    if (size < 2) {
        std::cerr << "警告: 窗口大小应大于等于2" << std::endl;
        return;
    }
    
    windowSize_ = size;
    
    // 调整所有窗口大小
    for (auto& window : windows_) {
        while (window.size() > windowSize_) {
            window.pop_front();
        }
    }
}

void TorqueSmoother::setPIDParams(const PIDParams& params) {
    pidParams_ = params;
}

void TorqueSmoother::setRateLimit(float max_rate, float sample_time) {
    if (max_rate <= 0.0f) {
        std::cerr << "警告: 最大变化率应大于0" << std::endl;
        return;
    }
    
    if (sample_time <= 0.0f) {
        std::cerr << "警告: 采样时间应大于0" << std::endl;
        return;
    }
    
    maxRateOfChange_ = max_rate;
    sampleTime_ = sample_time;
}

void TorqueSmoother::reset() {
    // 清空历史数据
    for (auto& window : windows_) {
        window.clear();
    }
    
    // 重置输出值
    std::fill(lastOutputs_.begin(), lastOutputs_.end(), 0.0f);
    
    // 重置PID相关数据
    std::fill(pidErrors_.begin(), pidErrors_.end(), 0.0f);
    std::fill(pidIntegrals_.begin(), pidIntegrals_.end(), 0.0f);
    std::fill(pidDerivatives_.begin(), pidDerivatives_.end(), 0.0f);
    
    // 重置时间戳
    uint64_t currentTime = getCurrentTimestamp();
    std::fill(lastUpdateTimes_.begin(), lastUpdateTimes_.end(), currentTime);
}

float TorqueSmoother::applyLowPassFilter(float current, float previous) const {
    // 一阶低通滤波: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    return alpha_ * current + (1.0f - alpha_) * previous;
}

float TorqueSmoother::applyMovingAverage(float value, std::deque<float>& window) {
    // 添加新值到窗口
    window.push_back(value);
    
    // 保持窗口大小不超过最大值
    if (window.size() > windowSize_) {
        window.pop_front();
    }
    
    // 计算平均值
    if (window.empty()) {
        return 0.0f;
    }
    
    float sum = std::accumulate(window.begin(), window.end(), 0.0f);
    return sum / window.size();
}

float TorqueSmoother::applyPIDControl(float setpoint, float current, size_t wheel_index) {
    // 获取当前时间
    uint64_t currentTime = getCurrentTimestamp();
    
    // 计算时间增量(秒)
    float dt = (currentTime - lastUpdateTimes_[wheel_index]) / 1000000.0f;
    
    // 防止时间增量过大或过小
    if (dt < 0.001f || dt > 1.0f) {
        dt = sampleTime_;
    }
    
    // 计算当前误差
    float error = setpoint - current;
    
    // 计算积分项
    pidIntegrals_[wheel_index] += error * dt;
    
    // 防止积分饱和
    pidIntegrals_[wheel_index] = std::min(std::max(pidIntegrals_[wheel_index], 
                                         pidParams_.outputMin / pidParams_.ki), 
                                         pidParams_.outputMax / pidParams_.ki);
    
    // 计算微分项
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pidErrors_[wheel_index]) / dt;
    }
    
    // 计算输出
    float output = pidParams_.kp * error + 
                   pidParams_.ki * pidIntegrals_[wheel_index] +
                   pidParams_.kd * derivative;
    
    // 限制输出范围
    output = std::min(std::max(output, pidParams_.outputMin), pidParams_.outputMax);
    
    // 更新状态
    pidErrors_[wheel_index] = error;
    pidDerivatives_[wheel_index] = derivative;
    lastUpdateTimes_[wheel_index] = currentTime;
    
    return output;
}

float TorqueSmoother::applyRateLimiter(float current, float previous) const {
    // 计算允许的最大变化
    float maxChange = maxRateOfChange_ * sampleTime_;
    
    // 限制变化速率
    if (current > previous + maxChange) {
        return previous + maxChange;
    }
    else if (current < previous - maxChange) {
        return previous - maxChange;
    }
    else {
        return current;
    }
}

uint64_t TorqueSmoother::getCurrentTimestamp() const {
    // 获取当前微秒级时间戳
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

} // namespace vehicle_control
} // namespace chassis_control 