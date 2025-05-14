/**
 * @file tire_model.cpp
 * @brief 轮胎模型类的实现
 * 
 * 本文件实现了轮胎模型类，用于模拟轮胎动态特性。
 */

#include "vehicle_control/tire_model.h"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace vehicle_control {

TireModel::TireModel(float mu, float radius, float width, float stiffness, float damping, float cornering_stiffness)
    : mu_(std::max(0.1f, std::min(mu, 1.0f))),  // 限制摩擦系数在0.1-1.0之间
      wheel_radius_(std::max(0.1f, radius)),    // 限制轮胎半径最小为0.1米
      wheel_width_(std::max(0.05f, width)),     // 限制轮胎宽度最小为0.05米
      wheel_stiffness_(std::max(1000.0f, stiffness)),  // 限制最小刚度
      wheel_damping_(std::max(100.0f, damping)),       // 限制最小阻尼
      cornering_stiffness_(std::max(1000.0f, cornering_stiffness)), // 限制最小侧偏刚度
      normal_force_(0.0f),
      slip_ratio_(0.0f),
      input_torque_(0.0f),
      longitudinal_force_(0.0f),
      lateral_force_(0.0f),
      slip_threshold_(0.2f),  // 默认滑移阈值
      is_sliding_(false),
      vertical_displacement_(0.0f) {
}

TireModel::~TireModel() {
    // 析构函数，无需特殊清理
}

void TireModel::updateLoad(float load) {
    // 确保负载为正值
    normal_force_ = std::max(0.0f, load);
    
    // 更新垂向位移
    if (wheel_stiffness_ > 0.0f) {
        vertical_displacement_ = normal_force_ / wheel_stiffness_;
    } else {
        vertical_displacement_ = 0.0f;
    }
}

void TireModel::updateSlipRatio(float slip_ratio) {
    // 限制滑移比在合理范围内
    slip_ratio_ = std::max(-1.0f, std::min(slip_ratio, 1.0f));
    
    // 根据滑移比更新滑动状态
    is_sliding_ = std::abs(slip_ratio_) > slip_threshold_;
}

void TireModel::updateTorqueInput(float torque) {
    input_torque_ = torque;
    
    // 如果有垂直负载，检查是否会导致滑动
    if (normal_force_ > 0.0f) {
        float max_torque = getMaxTractionForce() * wheel_radius_;
        is_sliding_ = is_sliding_ || (std::abs(input_torque_) > max_torque);
    }
}

float TireModel::computeLongitudinalForce() {
    // 如果没有垂直负载，无法产生纵向力
    if (normal_force_ <= 0.0f) {
        longitudinal_force_ = 0.0f;
        return longitudinal_force_;
    }
    
    // 计算最大静摩擦力
    float max_friction = getMaxTractionForce();
    
    // 计算理论纵向力（基于输入扭矩）
    float theoretical_force = input_torque_ / wheel_radius_;
    
    // 使用滑移曲线调整力
    float slip_factor = slipCurve(slip_ratio_);
    
    // 计算接触面积调整因子
    float contact_area_factor = std::sqrt(computeContactPatchArea()) / (M_PI * wheel_radius_ * 0.05f);
    contact_area_factor = std::min(1.5f, std::max(0.5f, contact_area_factor));
    
    if (is_sliding_) {
        // 如果滑动，使用动摩擦力（通常小于静摩擦力）
        float dynamic_mu = mu_ * 0.7f;  // 动摩擦系数假设为静摩擦系数的70%
        longitudinal_force_ = dynamic_mu * normal_force_ * slip_factor * contact_area_factor * (theoretical_force > 0 ? 1.0f : -1.0f);
    } else {
        // 如果没有滑动，使用静摩擦力，但不超过最大静摩擦力
        longitudinal_force_ = std::min(std::abs(theoretical_force), max_friction) * (theoretical_force > 0 ? 1.0f : -1.0f);
        longitudinal_force_ *= slip_factor * contact_area_factor;
    }
    
    return longitudinal_force_;
}

float TireModel::computeLateralForce(float slip_angle) {
    // 如果没有垂直负载，无法产生横向力
    if (normal_force_ <= 0.0f) {
        lateral_force_ = 0.0f;
        return lateral_force_;
    }
    
    // 小角度情况下，横向力与侧偏角成线性关系
    if (std::abs(slip_angle) < 0.1f) {
        lateral_force_ = -cornering_stiffness_ * slip_angle;
    } else {
        // 大角度下，使用简化的饱和模型
        float max_lateral_force = mu_ * normal_force_;
        float sign = (slip_angle > 0.0f) ? 1.0f : -1.0f;
        float saturation_factor = 1.0f - std::exp(-(std::abs(slip_angle) - 0.1f) / 0.2f);
        lateral_force_ = -sign * (cornering_stiffness_ * 0.1f + saturation_factor * (max_lateral_force - cornering_stiffness_ * 0.1f));
    }
    
    // 考虑滑动状态
    if (is_sliding_) {
        lateral_force_ *= 0.7f;  // 滑动时横向力降低
    }
    
    return lateral_force_;
}

float TireModel::computeVerticalDisplacement() {
    // 使用弹簧-阻尼模型计算垂向位移
    // 这是一个简化模型，实际中可能需要考虑非线性因素
    if (wheel_stiffness_ > 0.0f) {
        return normal_force_ / wheel_stiffness_;
    }
    return 0.0f;
}

bool TireModel::isSliding() const {
    return is_sliding_;
}

void TireModel::setFrictionCoefficient(float mu) {
    mu_ = std::max(0.1f, std::min(mu, 1.0f));
}

void TireModel::setSlipThreshold(float threshold) {
    slip_threshold_ = std::max(0.01f, std::min(threshold, 0.5f));
    
    // 更新滑动状态
    is_sliding_ = std::abs(slip_ratio_) > slip_threshold_;
}

float TireModel::getMaxTractionForce() const {
    return mu_ * normal_force_;
}

bool TireModel::getSlideState() const {
    return is_sliding_;
}

float TireModel::getRadius() const {
    return wheel_radius_;
}

void TireModel::setRadius(float radius) {
    wheel_radius_ = std::max(0.1f, radius);
}

float TireModel::getWidth() const {
    return wheel_width_;
}

void TireModel::setWidth(float width) {
    wheel_width_ = std::max(0.05f, width);
}

float TireModel::getStiffness() const {
    return wheel_stiffness_;
}

void TireModel::setStiffness(float stiffness) {
    wheel_stiffness_ = std::max(1000.0f, stiffness);
    
    // 更新垂向位移
    if (wheel_stiffness_ > 0.0f && normal_force_ > 0.0f) {
        vertical_displacement_ = normal_force_ / wheel_stiffness_;
    }
}

float TireModel::getDamping() const {
    return wheel_damping_;
}

void TireModel::setDamping(float damping) {
    wheel_damping_ = std::max(100.0f, damping);
}

float TireModel::getCorneringStiffness() const {
    return cornering_stiffness_;
}

void TireModel::setCorneringStiffness(float cornering_stiffness) {
    cornering_stiffness_ = std::max(1000.0f, cornering_stiffness);
}

float TireModel::computeContactPatchArea() const {
    // 计算轮胎接触面积
    // 使用简化模型：面积与垂向位移和轮胎宽度相关
    if (normal_force_ <= 0.0f || wheel_stiffness_ <= 0.0f) {
        return 0.0f;
    }
    
    // 计算接触面贴长度（假设轮胎变形接近圆弧）
    float contact_length = 2.0f * std::sqrt(wheel_radius_ * vertical_displacement_ - vertical_displacement_ * vertical_displacement_ / 4.0f);
    
    // 计算接触面积
    return contact_length * wheel_width_;
}

float TireModel::getSlipRatio() const {
    return slip_ratio_;
}

float TireModel::getNormalForce() const {
    return normal_force_;
}

float TireModel::getInputTorque() const {
    return input_torque_;
}

float TireModel::getLongitudinalForce() const {
    return longitudinal_force_;
}

float TireModel::slipCurve(float slip) {
    // 简化的滑移曲线模型
    // 在小滑移比时，摩擦系数随滑移比线性增加
    // 达到峰值后，摩擦系数随滑移比增大而减小
    
    float abs_slip = std::abs(slip);
    
    // 经典的"魔术公式"(Magic Formula)的简化版本
    if (abs_slip < 0.1f) {
        // 线性区域
        return abs_slip / 0.1f;
    } else if (abs_slip < 0.2f) {
        // 峰值区域
        return 1.0f;
    } else {
        // 滑移区域
        return 1.0f - 0.3f * (abs_slip - 0.2f) / 0.8f;
    }
}

} // namespace vehicle_control 