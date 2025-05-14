/**
 * @file state_estimator.cpp
 * @brief 状态估计器类实现
 */

#include "data_input/state_estimator.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>
#include <chrono>

namespace chassis_control {
namespace data_input {

StateEstimator::StateEstimator() : 
    alpha(0.1f),
    windowSize(10),
    smoothSpeed(0.0f),
    smoothAccel(0.0f),
    estimatedAttitude(6, 0.0f),
    lastUpdateTime(getCurrentTimestamp()) {
    
    // 初始化姿态窗口
    attitudeWindows.resize(6);  // 6个姿态数据通道
}

StateEstimator::StateEstimator(float filter_alpha, size_t window_size) : 
    alpha(filter_alpha),
    windowSize(window_size),
    smoothSpeed(0.0f),
    smoothAccel(0.0f),
    estimatedAttitude(6, 0.0f),
    lastUpdateTime(getCurrentTimestamp()) {
    
    // 参数检查
    if (alpha < 0.0f || alpha > 1.0f) {
        alpha = 0.1f;  // 使用默认值
        std::cerr << "警告: 滤波系数应在0-1范围内，已重置为默认值0.1" << std::endl;
    }
    
    if (windowSize < 2) {
        windowSize = 10;  // 使用默认值
        std::cerr << "警告: 窗口大小应大于等于2，已重置为默认值10" << std::endl;
    }
    
    // 初始化姿态窗口
    attitudeWindows.resize(6);  // 6个姿态数据通道
}

StateEstimator::~StateEstimator() {
    // 清理资源(如有必要)
}

void StateEstimator::update(float speed, float accel) {
    // 更新时间戳
    uint64_t currentTime = getCurrentTimestamp();
    
    // 更新滑动窗口
    updateWindow(speedWindow, speed);
    updateWindow(accelWindow, accel);
    
    // 应用低通滤波
    smoothSpeed = applyLowPassFilter(speed, smoothSpeed);
    smoothAccel = applyLowPassFilter(accel, smoothAccel);
    
    // 更新时间戳
    lastUpdateTime = currentTime;
}

void StateEstimator::update(float speed, float accel, const std::vector<float>& attitude) {
    // 基本更新
    update(speed, accel);
    
    // 检查姿态数据有效性
    if (attitude.size() != 6) {
        std::cerr << "警告: 姿态数据应包含6个元素，实际接收到" << attitude.size() << "个" << std::endl;
        return;
    }
    
    // 更新姿态数据窗口
    for (size_t i = 0; i < attitude.size(); ++i) {
        updateWindow(attitudeWindows[i], attitude[i]);
        
        // 应用低通滤波
        estimatedAttitude[i] = applyLowPassFilter(attitude[i], estimatedAttitude[i]);
    }
}

float StateEstimator::getFilteredSpeed() const {
    return smoothSpeed;
}

float StateEstimator::getFilteredAcceleration() const {
    return smoothAccel;
}

std::vector<float> StateEstimator::getEstimatedAttitude() const {
    return estimatedAttitude;
}

void StateEstimator::setFilterAlpha(float new_alpha) {
    if (new_alpha < 0.0f || new_alpha > 1.0f) {
        std::cerr << "警告: 滤波系数应在0-1范围内" << std::endl;
        return;
    }
    alpha = new_alpha;
}

void StateEstimator::setWindowSize(size_t size) {
    if (size < 2) {
        std::cerr << "警告: 窗口大小应大于等于2" << std::endl;
        return;
    }
    windowSize = size;
    
    // 调整窗口大小
    while (speedWindow.size() > windowSize) {
        speedWindow.pop_front();
    }
    
    while (accelWindow.size() > windowSize) {
        accelWindow.pop_front();
    }
    
    for (auto& window : attitudeWindows) {
        while (window.size() > windowSize) {
            window.pop_front();
        }
    }
}

void StateEstimator::reset() {
    // 重置所有滤波后的状态
    smoothSpeed = 0.0f;
    smoothAccel = 0.0f;
    std::fill(estimatedAttitude.begin(), estimatedAttitude.end(), 0.0f);
    
    // 清空数据窗口
    speedWindow.clear();
    accelWindow.clear();
    for (auto& window : attitudeWindows) {
        window.clear();
    }
    
    // 重置时间戳
    lastUpdateTime = getCurrentTimestamp();
}

void StateEstimator::printState() const {
    std::cout << "===== 状态估计器当前状态 =====" << std::endl;
    std::cout << "滤波速度: " << smoothSpeed << " m/s" << std::endl;
    std::cout << "滤波加速度: " << smoothAccel << " m/s^2" << std::endl;
    std::cout << "估计姿态: [";
    for (size_t i = 0; i < estimatedAttitude.size(); ++i) {
        std::cout << estimatedAttitude[i];
        if (i < estimatedAttitude.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
    std::cout << "滤波参数 - alpha: " << alpha << ", 窗口大小: " << windowSize << std::endl;
    std::cout << "================================" << std::endl;
}

uint64_t StateEstimator::getCurrentTimestamp() const {
    // 获取当前微秒级时间戳
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

float StateEstimator::applyLowPassFilter(float current, float previous) const {
    // 一阶低通滤波: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    return alpha * current + (1.0f - alpha) * previous;
}

float StateEstimator::applyMovingAverageFilter(const std::deque<float>& window) const {
    if (window.empty()) {
        return 0.0f;
    }
    
    // 计算移动平均值
    float sum = std::accumulate(window.begin(), window.end(), 0.0f);
    return sum / window.size();
}

void StateEstimator::updateWindow(std::deque<float>& window, float value) {
    // 添加新值到窗口
    window.push_back(value);
    
    // 如果窗口超过指定大小，则移除最旧的值
    if (window.size() > windowSize) {
        window.pop_front();
    }
}

} // namespace data_input
} // namespace chassis_control 