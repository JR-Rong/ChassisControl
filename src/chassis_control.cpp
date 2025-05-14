/**
 * @file chassis_control.cpp
 * @brief 底盘控制系统高级接口实现
 * 
 * 实现chassis_control.h中定义的高级接口，连接底层模块实现底盘控制功能
 */

#include "chassis_control.h"

// 包含所需的底层模块头文件
#include "data_input/vehicle_config.h"
#include "data_input/state_estimator.h"
#include "vehicle_control/vehicle_model.h"
#include "vehicle_control/torque_allocator.h"
#include "vehicle_control/torque_smoother.h"
#include "vehicle_control/tire_model.h"
#include "data_output/can_bus_interface.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <yaml-cpp/yaml.h>

namespace chassis_control {

/**
 * @brief 底盘控制实现类
 * 
 * 具体实现chassis_control模块的功能，封装底层模块
 */
class ChassisControlImpl {
public:
    ChassisControlImpl() 
        : initialized_(false),
          control_mode_(TORQUE_CONTROL),
          current_speed_(0.0f),
          current_acceleration_(0.0f),
          current_steering_angle_(0.0f) {
    }
    
    ~ChassisControlImpl() {
        // 在析构时关闭CAN接口
        if (can_interface_ && can_interface_->isOpen()) {
            can_interface_->close();
        }
    }
    
    bool loadConfig(const std::string& config_file) {
        try {
            // 使用VehicleConfig加载配置
            config_ = std::make_unique<data_input::VehicleConfig>();
            if (!config_->loadFromFile(config_file)) {
                std::cerr << "加载配置文件失败: " << config_file << std::endl;
                return false;
            }
            
            // 读取车辆参数
            vehicle_mass_ = config_->getVehicleMass();
            wheel_radius_ = config_->getWheelRadius();
            wheel_base_ = config_->getWheelBase();
            track_width_ = config_->getTrackWidth();
            max_steering_angle_ = config_->getMaxSteeringAngle();
            max_torque_ = config_->getMaxTorque();
            can_channel_ = config_->getCANChannel();
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "加载配置异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool initialize(float vehicle_mass, float wheel_radius, float wheel_base, 
                     float track_width, float max_steering_angle, float max_torque,
                     const std::string& can_channel) {
        
        if (initialized_) return true;  // 已初始化，直接返回成功
        
        // 记录参数
        vehicle_mass_ = std::max(100.0f, vehicle_mass);  // 至少100kg
        wheel_radius_ = std::max(0.1f, wheel_radius);    // 至少10cm
        wheel_base_ = std::max(0.5f, wheel_base);        // 至少0.5m
        track_width_ = std::max(0.5f, track_width);      // 至少0.5m
        max_steering_angle_ = std::clamp(max_steering_angle, 1.0f, 70.0f);  // 限制在1-70度
        max_torque_ = std::max(10.0f, max_torque);       // 至少10Nm
        can_channel_ = can_channel;
        
        std::cout << "初始化底盘控制系统..." << std::endl
                  << "车辆质量: " << vehicle_mass_ << " kg" << std::endl
                  << "轮胎半径: " << wheel_radius_ << " m" << std::endl
                  << "轴距: " << wheel_base_ << " m" << std::endl
                  << "轮距: " << track_width_ << " m" << std::endl
                  << "最大转向角: " << max_steering_angle_ << " 度" << std::endl
                  << "最大扭矩: " << max_torque_ << " N·m" << std::endl
                  << "CAN通道: " << can_channel_ << std::endl;
        
        try {
            // 创建组件实例
            vehicle_model_ = std::make_unique<vehicle_control::VehicleModel>(
                wheel_base_, track_width_, vehicle_mass_);
            
            allocator_ = std::make_unique<vehicle_control::TorqueAllocator>(
                vehicle_mass_, wheel_radius_, max_torque_);
            
            can_interface_ = std::make_unique<vehicle_control::CANBusInterface>(
                can_channel_);
            
            // 创建轮胎模型
            tire_models_.resize(4);  // 默认4个轮胎
            float default_tire_load = vehicle_mass_ * 9.81f / 4.0f;  // 默认均匀分布
            
            for (int i = 0; i < 4; i++) {
                tire_models_[i] = std::make_unique<vehicle_control::TireModel>(
                    wheel_radius_,       // 轮胎半径
                    0.7f,                // 静摩擦系数(默认干燥路面)
                    default_tire_load,   // 垂直载荷
                    0.0f,                // 初始滑移比
                    0.0f,                // 初始扭矩输入
                    0.225f,              // 默认轮胎宽度 (m)
                    250000.0f,           // 默认轮胎刚度 (N/m)
                    5000.0f,             // 默认轮胎阻尼系数 (N·s/m)
                    80000.0f             // 默认侧偏刚度 (N/rad)
                );
            }
            
            // 设置默认控制模式、分配器和平滑器
            control_mode_ = ControlMode::TORQUE_CONTROL;
            allocator_type_ = AllocatorType::DYNAMIC_ALLOCATOR;
            smoother_type_ = SmootherType::QUADRATIC_SMOOTHER;
            
            // 初始化车轮状态
            wheel_states_.resize(4);
            wheel_torques_.resize(4, 0.0f);
            
            // 初始化平滑器
            smoother_ = std::make_unique<vehicle_control::TorqueSmoother>();
            
            // 标记初始化完成
            initialized_ = true;
            std::cout << "底盘控制系统初始化成功" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "初始化失败: " << e.what() << std::endl;
            return false;
        }
    }
    
    void setControlMode(ControlMode mode) {
        control_mode_ = mode;
    }
    
    void setTargetSpeed(float speed) {
        target_speed_ = std::clamp(speed, -max_speed_, max_speed_);
    }
    
    void setTargetAcceleration(float acceleration) {
        target_acceleration_ = std::clamp(acceleration, -max_acceleration_, max_acceleration_);
    }
    
    void setSteeringAngle(float angle) {
        // 将角度转换为弧度，并限制在最大转向角范围内
        float angle_rad = angle * M_PI / 180.0f;
        float max_rad = max_steering_angle_ * M_PI / 180.0f;
        current_steering_angle_ = std::clamp(angle_rad, -max_rad, max_rad);
    }
    
    void setAllocatorType(AllocatorType type) {
        if (!allocator_) return;
        
        // 转换为内部枚举类型
        allocator_type_ = type;
        vehicle_control::TorqueAllocator::Method method;
        
        switch (type) {
            case AllocatorType::PROPORTIONAL_ALLOCATOR:
                method = vehicle_control::TorqueAllocator::Method::PROPORTIONAL;
                break;
            case AllocatorType::DYNAMIC_ALLOCATOR:
                method = vehicle_control::TorqueAllocator::Method::DYNAMIC;
                break;
            case AllocatorType::OPTIMIZATION_ALLOCATOR:
                method = vehicle_control::TorqueAllocator::Method::OPTIMIZATION;
                break;
            default:
                method = vehicle_control::TorqueAllocator::Method::PROPORTIONAL;
        }
        
        allocator_->setMethod(method);
    }
    
    void setSmootherType(SmootherType type) {
        if (!smoother_) return;
        
        // 转换为内部枚举类型
        smoother_type_ = type;
        vehicle_control::TorqueSmoother::Type smoother_type;
        
        switch (type) {
            case SmootherType::NO_SMOOTHING:
                smoother_type = vehicle_control::TorqueSmoother::Type::NONE;
                break;
            case SmootherType::LOW_PASS_FILTER:
                smoother_type = vehicle_control::TorqueSmoother::Type::LOW_PASS_FILTER;
                break;
            case SmootherType::MOVING_AVERAGE:
                smoother_type = vehicle_control::TorqueSmoother::Type::MOVING_AVERAGE;
                break;
            case SmootherType::PID_CONTROLLER:
                smoother_type = vehicle_control::TorqueSmoother::Type::PID;
                break;
            case SmootherType::QUADRATIC_SMOOTHER:
                smoother_type = vehicle_control::TorqueSmoother::Type::QUADRATIC;
                break;
            default:
                smoother_type = vehicle_control::TorqueSmoother::Type::LOW_PASS_FILTER;
        }
        
        smoother_->setType(smoother_type);
    }
    
    void setPIDParameters(float kp, float ki, float kd) {
        if (!smoother_) return;
        
        vehicle_control::TorqueSmoother::PIDParams params;
        params.kp = kp;
        params.ki = ki;
        params.kd = kd;
        
        smoother_->setPIDParams(params);
    }
    
    void setLowPassFilterAlpha(float alpha) {
        if (!smoother_) return;
        smoother_->setFilterAlpha(alpha);
    }
    
    void setMovingAverageWindowSize(size_t window_size) {
        if (!smoother_) return;
        smoother_->setWindowSize(window_size);
    }
    
    void setRateLimiter(float max_rate) {
        if (!smoother_) return;
        smoother_->setMaxRateOfChange(max_rate);
    }
    
    bool updateVehicleState(float speed, float acceleration, float steering_angle) {
        if (!initialized_) return false;
        
        try {
            // 更新状态估计器
            estimator_->updateSpeed(speed);
            estimator_->updateAcceleration(acceleration);
            estimator_->updateSteeringAngle(steering_angle);
            
            // 获取过滤后的状态
            current_speed_ = estimator_->getFilteredSpeed();
            current_acceleration_ = estimator_->getFilteredAcceleration();
            current_steering_angle_ = estimator_->getFilteredSteeringAngle();
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "更新车辆状态异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool control(float target_speed, float target_acceleration, float target_steering_angle) {
        if (!initialized_) return false;
        
        try {
            // 1. 计算车轮状态
            auto steering_function = vehicle_model_->getDefaultAckermannFunction();
            wheel_states_ = vehicle_model_->computeWheelStates(
                current_speed_,
                current_acceleration_,
                target_steering_angle,
                steering_function
            );
            
            // 2. 根据控制模式和目标计算所需总扭矩
            float total_torque = 0.0f;
            switch (control_mode_) {
                case SPEED_CONTROL: {
                    // 使用简单的PID控制计算扭矩
                    float speed_error = target_speed - current_speed_;
                    float p_term = 20.0f * speed_error;  // 比例项
                    float feedforward = vehicle_mass_ * wheel_radius_ * target_acceleration;
                    total_torque = p_term + feedforward;
                    break;
                }
                case TORQUE_CONTROL: {
                    // 直接使用目标加速度计算扭矩
                    total_torque = vehicle_mass_ * wheel_radius_ * target_acceleration;
                    break;
                }
                case POSITION_CONTROL:
                    // 位置控制模式暂不实现
                    std::cerr << "位置控制模式未实现" << std::endl;
                    return false;
            }
            
            // 限制总扭矩
            total_torque = std::max(-max_torque_, std::min(total_torque, max_torque_));
            
            // 3. 分配扭矩
            wheel_torques_ = allocator_->allocateTorque(
                total_torque,
                current_steering_angle_,
                0.0f  // 目标横摆力矩，暂时为0
            );
            
            // 4. 平滑扭矩
            wheel_torques_ = smoother_->smoothTorques(wheel_torques_);
            
            // 5. 更新轮胎模型
            for (size_t i = 0; i < wheel_states_.size(); ++i) {
                if (i < tire_models_.size()) {
                    // 更新轮胎属性
                    float load = vehicle_mass_ * 9.81f / 4.0f;  // 简单假设均匀负载
                    tire_models_[i]->updateLoad(load);
                    tire_models_[i]->updateSlipRatio(wheel_states_[i].slipRatio);
                    tire_models_[i]->updateTorqueInput(wheel_torques_[i]);
                    
                    // 计算轮胎力
                    float force = tire_models_[i]->computeLongitudinalForce();
                    
                    // 检查轮胎滑动状态
                    if (tire_models_[i]->isSliding()) {
                        std::cout << "警告: 轮胎 " << i << " 滑动!" << std::endl;
                        // 可以在这里实现防滑控制
                    }
                }
            }
            
            // 6. 发送控制命令到CAN
            if (can_interface_ && can_interface_->isOpen()) {
                // 创建扭矩控制消息
                data_output::CANFrame torque_frame;
                torque_frame.id = 0x200;  // 扭矩控制消息ID
                torque_frame.dlc = 8;
                
                // 简单地将扭矩值打包到CAN帧中
                // 这里只是示例，实际的打包逻辑取决于底层控制器的协议
                for (size_t i = 0; i < 4 && i < wheel_torques_.size(); ++i) {
                    int16_t torque_value = static_cast<int16_t>(wheel_torques_[i] * 10.0f);  // 扩大10倍，提高精度
                    torque_frame.data[i*2] = (torque_value >> 8) & 0xFF;  // 高字节
                    torque_frame.data[i*2+1] = torque_value & 0xFF;       // 低字节
                }
                
                // 创建转向控制消息
                data_output::CANFrame steering_frame;
                steering_frame.id = 0x201;  // 转向控制消息ID
                steering_frame.dlc = 2;
                
                // 将转向角打包到CAN帧中
                int16_t steering_value = static_cast<int16_t>(target_steering_angle * 1000.0f);  // 扩大1000倍，提高精度
                steering_frame.data[0] = (steering_value >> 8) & 0xFF;  // 高字节
                steering_frame.data[1] = steering_value & 0xFF;         // 低字节
                
                // 发送帧
                if (!can_interface_->sendFrame(torque_frame) || !can_interface_->sendFrame(steering_frame)) {
                    std::cerr << "发送CAN消息失败" << std::endl;
                    return false;
                }
            } else {
                std::cerr << "CAN接口未打开" << std::endl;
                return false;
            }
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "控制异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool emergencyStop() {
        if (!initialized_) return false;
        
        try {
            // 将所有轮扭矩设为0
            wheel_torques_.assign(wheel_torques_.size(), 0.0f);
            
            // 重置平滑器状态
            if (smoother_) {
                smoother_->reset();
            }
            
            // 发送紧急停止消息到CAN
            if (can_interface_ && can_interface_->isOpen()) {
                data_output::CANFrame emergency_frame;
                emergency_frame.id = 0x202;  // 紧急停止消息ID
                emergency_frame.dlc = 1;
                emergency_frame.data[0] = 0x01;  // 紧急停止标志
                
                if (!can_interface_->sendFrame(emergency_frame)) {
                    std::cerr << "发送紧急停止消息失败" << std::endl;
                    return false;
                }
            } else {
                std::cerr << "CAN接口未打开" << std::endl;
                return false;
            }
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "紧急停止异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    std::vector<float> getWheelTorques() const {
        return wheel_torques_;
    }
    
    float getCurrentSpeed() const {
        return current_speed_;
    }
    
    float getCurrentAcceleration() const {
        return current_acceleration_;
    }
    
    float getCurrentSteeringAngle() const {
        return current_steering_angle_;
    }
    
    bool sendDiagnostics() {
        if (!initialized_ || !can_interface_ || !can_interface_->isOpen()) {
            return false;
        }
        
        try {
            // 创建诊断消息
            data_output::CANFrame diag_frame;
            diag_frame.id = 0x203;  // 诊断消息ID
            diag_frame.dlc = 8;
            
            // 填充诊断数据
            int16_t speed_value = static_cast<int16_t>(current_speed_ * 100.0f);  // 扩大100倍
            int16_t accel_value = static_cast<int16_t>(current_acceleration_ * 100.0f);  // 扩大100倍
            int16_t steering_value = static_cast<int16_t>(current_steering_angle_ * 1000.0f);  // 扩大1000倍
            
            diag_frame.data[0] = (speed_value >> 8) & 0xFF;
            diag_frame.data[1] = speed_value & 0xFF;
            diag_frame.data[2] = (accel_value >> 8) & 0xFF;
            diag_frame.data[3] = accel_value & 0xFF;
            diag_frame.data[4] = (steering_value >> 8) & 0xFF;
            diag_frame.data[5] = steering_value & 0xFF;
            
            // 添加状态标志
            diag_frame.data[6] = static_cast<uint8_t>(control_mode_);
            diag_frame.data[7] = initialized_ ? 0x01 : 0x00;
            
            if (!can_interface_->sendFrame(diag_frame)) {
                std::cerr << "发送诊断消息失败" << std::endl;
                return false;
            }
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "发送诊断异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    void setTireFrictionCoefficient(float mu, int wheel_index) {
        if (!initialized_) return;
        
        if (wheel_index < 0) {
            // 设置所有轮胎的摩擦系数
            for (auto& tire : tire_models_) {
                if (tire) {
                    tire->setFrictionCoefficient(mu);
                }
            }
        } else if (wheel_index < static_cast<int>(tire_models_.size())) {
            // 设置指定轮胎的摩擦系数
            if (tire_models_[wheel_index]) {
                tire_models_[wheel_index]->setFrictionCoefficient(mu);
            }
        }
    }
    
    void setTireLoad(float load, int wheel_index) {
        if (!initialized_) return;
        
        if (wheel_index < 0) {
            // 设置所有轮胎的加载力
            for (auto& tire : tire_models_) {
                if (tire) {
                    tire->updateLoad(load);
                }
            }
        } else if (wheel_index < static_cast<int>(tire_models_.size())) {
            // 设置指定轮胎的加载力
            if (tire_models_[wheel_index]) {
                tire_models_[wheel_index]->updateLoad(load);
            }
        }
    }
    
    void setTireSlipThreshold(float threshold, int wheel_index) {
        if (!initialized_) return;
        
        if (wheel_index < 0) {
            // 设置所有轮胎的滑移阈值
            for (auto& tire : tire_models_) {
                if (tire) {
                    tire->setSlipThreshold(threshold);
                }
            }
        } else if (wheel_index < static_cast<int>(tire_models_.size())) {
            // 设置指定轮胎的滑移阈值
            if (tire_models_[wheel_index]) {
                tire_models_[wheel_index]->setSlipThreshold(threshold);
            }
        }
    }
    
    bool isTireSliding(int wheel_index) const {
        if (!initialized_ || wheel_index < 0 || wheel_index >= static_cast<int>(tire_models_.size())) {
            return false;
        }
        
        return tire_models_[wheel_index] ? tire_models_[wheel_index]->isSliding() : false;
    }
    
    float getTireSlipRatio(int wheel_index) const {
        if (!initialized_ || wheel_index < 0 || wheel_index >= static_cast<int>(tire_models_.size())) {
            return 0.0f;
        }
        
        return tire_models_[wheel_index] ? tire_models_[wheel_index]->getSlipRatio() : 0.0f;
    }
    
    float getTireLongitudinalForce(int wheel_index) const {
        if (!initialized_ || wheel_index < 0 || wheel_index >= static_cast<int>(tire_models_.size())) {
            return 0.0f;
        }
        
        return tire_models_[wheel_index] ? tire_models_[wheel_index]->getLongitudinalForce() : 0.0f;
    }
    
    float getTireLoad(int wheel_index) const {
        if (!initialized_ || wheel_index < 0 || wheel_index >= static_cast<int>(tire_models_.size())) {
            return 0.0f;
        }
        
        return tire_models_[wheel_index] ? tire_models_[wheel_index]->getNormalForce() : 0.0f;
    }
    
    void setTireRadius(float radius, int wheel_index) {
        if (!initialized_) return;
        
        if (wheel_index < 0) {
            // 设置所有轮胎的半径
            for (auto& tire : tire_models_) {
                if (tire) {
                    tire->setRadius(radius);
                }
            }
        } else if (wheel_index < static_cast<int>(tire_models_.size())) {
            // 设置指定轮胎的半径
            if (tire_models_[wheel_index]) {
                tire_models_[wheel_index]->setRadius(radius);
            }
        }
    }
    
    void setTireWidth(float width, int wheel_index) {
        if (!initialized_) return;
        
        if (wheel_index < 0) {
            // 设置所有轮胎的宽度
            for (auto& tire : tire_models_) {
                if (tire) {
                    tire->setWidth(width);
                }
            }
        } else if (wheel_index < static_cast<int>(tire_models_.size())) {
            // 设置指定轮胎的宽度
            if (tire_models_[wheel_index]) {
                tire_models_[wheel_index]->setWidth(width);
            }
        }
    }
    
    void setTireStiffness(float stiffness, int wheel_index) {
        if (!initialized_) return;
        
        if (wheel_index < 0) {
            // 设置所有轮胎的刚度
            for (auto& tire : tire_models_) {
                if (tire) {
                    tire->setStiffness(stiffness);
                }
            }
        } else if (wheel_index < static_cast<int>(tire_models_.size())) {
            // 设置指定轮胎的刚度
            if (tire_models_[wheel_index]) {
                tire_models_[wheel_index]->setStiffness(stiffness);
            }
        }
    }
    
    void setTireDamping(float damping, int wheel_index) {
        if (!initialized_) return;
        
        if (wheel_index < 0) {
            // 设置所有轮胎的阻尼系数
            for (auto& tire : tire_models_) {
                if (tire) {
                    tire->setDamping(damping);
                }
            }
        } else if (wheel_index < static_cast<int>(tire_models_.size())) {
            // 设置指定轮胎的阻尼系数
            if (tire_models_[wheel_index]) {
                tire_models_[wheel_index]->setDamping(damping);
            }
        }
    }
    
    void setTireCorneringStiffness(float cornering_stiffness, int wheel_index) {
        if (!initialized_) return;
        
        if (wheel_index < 0) {
            // 设置所有轮胎的侧偏刚度
            for (auto& tire : tire_models_) {
                if (tire) {
                    tire->setCorneringStiffness(cornering_stiffness);
                }
            }
        } else if (wheel_index < static_cast<int>(tire_models_.size())) {
            // 设置指定轮胎的侧偏刚度
            if (tire_models_[wheel_index]) {
                tire_models_[wheel_index]->setCorneringStiffness(cornering_stiffness);
            }
        }
    }
    
    float getTireLateralForce(int wheel_index) const {
        if (!initialized_ || wheel_index < 0 || wheel_index >= static_cast<int>(tire_models_.size())) {
            return 0.0f;
        }
        
        // 简化处理，假设侧偏角与转向角相关
        float slip_angle = 0.0f;
        if (wheel_index < static_cast<int>(wheel_states_.size())) {
            slip_angle = wheel_states_[wheel_index].steeringAngle * 0.5f;  // 简化计算
        }
        
        return tire_models_[wheel_index] ? tire_models_[wheel_index]->computeLateralForce(slip_angle) : 0.0f;
    }
    
    float getTireContactPatchArea(int wheel_index) const {
        if (!initialized_ || wheel_index < 0 || wheel_index >= static_cast<int>(tire_models_.size())) {
            return 0.0f;
        }
        
        return tire_models_[wheel_index] ? tire_models_[wheel_index]->computeContactPatchArea() : 0.0f;
    }
    
    float getTireVerticalDisplacement(int wheel_index) const {
        if (!initialized_ || wheel_index < 0 || wheel_index >= static_cast<int>(tire_models_.size())) {
            return 0.0f;
        }
        
        return tire_models_[wheel_index] ? tire_models_[wheel_index]->computeVerticalDisplacement() : 0.0f;
    }
    
    float getTireRadius(int wheel_index) const {
        if (!initialized_ || wheel_index < 0 || wheel_index >= static_cast<int>(tire_models_.size())) {
            return 0.0f;
        }
        
        return tire_models_[wheel_index] ? tire_models_[wheel_index]->getRadius() : 0.0f;
    }
    
    float getTireWidth(int wheel_index) const {
        if (!initialized_ || wheel_index < 0 || wheel_index >= static_cast<int>(tire_models_.size())) {
            return 0.0f;
        }
        
        return tire_models_[wheel_index] ? tire_models_[wheel_index]->getWidth() : 0.0f;
    }

    void update() {
        if (!initialized_) return;
        
        try {
            // 1. 更新车轮状态 (基于当前速度和转向角计算每个车轮的状态)
            wheel_states_ = vehicle_model_->computeWheelStates(
                current_speed_,
                current_acceleration_,
                current_steering_angle_,
                vehicle_model_->getDefaultAckermannSteeringFunction()
            );
            
            // 2. 计算所需的总扭矩
            float total_torque = 0.0f;
            switch (control_mode_) {
                case ControlMode::TORQUE_CONTROL: {
                    // 直接使用目标扭矩
                    total_torque = target_torque_;
                    break;
                }
                case ControlMode::SPEED_CONTROL: {
                    // 根据速度误差计算扭矩 (简单的PID)
                    float speed_error = target_speed_ - current_speed_;
                    total_torque = speed_error * speed_kp_ + target_acceleration_ * speed_kd_;
                    break;
                }
                case ControlMode::ACCELERATION_CONTROL: {
                    // 根据加速度计算扭矩 (F = m*a)
                    total_torque = vehicle_mass_ * target_acceleration_ * wheel_radius_;
                    break;
                }
            }
            
            // 3. 分配扭矩
            wheel_torques_ = allocator_->allocateTorque(
                total_torque,
                current_steering_angle_,
                wheel_states_
            );
            
            // 4. 平滑扭矩
            wheel_torques_ = smoother_->smoothTorques(wheel_torques_);
            
            // 5. 更新轮胎模型
            for (size_t i = 0; i < std::min(tire_models_.size(), wheel_states_.size()); ++i) {
                if (tire_models_[i]) {
                    // 更新轮胎载荷
                    tire_models_[i]->updateLoad(wheel_states_[i].normalForce);
                    
                    // 更新滑移比
                    float slip_ratio = wheel_states_[i].slipRatio;
                    tire_models_[i]->updateSlipRatio(slip_ratio);
                    
                    // 更新扭矩输入
                    tire_models_[i]->updateTorqueInput(wheel_torques_[i]);
                    
                    // 计算纵向力
                    float longitudinal_force = tire_models_[i]->computeLongitudinalForce();
                    wheel_states_[i].longitudinalForce = longitudinal_force;
                }
            }
            
            // 6. 发送控制命令到CAN总线
            if (can_interface_) {
                can_interface_->sendTorqueCommands(wheel_torques_);
            }
        } catch (const std::exception& e) {
            std::cerr << "更新控制器时发生错误: " << e.what() << std::endl;
        }
    }
    
private:
    // 状态标志
    bool initialized_ = false;
    
    // 车辆参数
    float vehicle_mass_ = 1500.0f;       // 车辆质量 (kg)
    float wheel_radius_ = 0.33f;         // 轮胎半径 (m)
    float wheel_base_ = 2.8f;            // 轴距 (m)
    float track_width_ = 1.8f;           // 轮距 (m)
    float max_steering_angle_ = 30.0f;   // 最大转向角 (度)
    float max_torque_ = 500.0f;          // 最大扭矩 (N·m)
    
    // 限制参数
    float max_speed_ = 40.0f;            // 最大速度 (m/s)
    float max_acceleration_ = 5.0f;      // 最大加速度 (m/s²)
    
    // 控制参数
    ControlMode control_mode_ = ControlMode::TORQUE_CONTROL;
    AllocatorType allocator_type_ = AllocatorType::PROPORTIONAL_ALLOCATOR;
    SmootherType smoother_type_ = SmootherType::LOW_PASS_FILTER;
    
    // 速度控制器参数
    float speed_kp_ = 200.0f;  // 速度比例系数
    float speed_ki_ = 50.0f;   // 速度积分系数
    float speed_kd_ = 20.0f;   // 速度微分系数
    
    // 目标状态
    float target_speed_ = 0.0f;          // 目标速度 (m/s)
    float target_acceleration_ = 0.0f;   // 目标加速度 (m/s²)
    float target_torque_ = 0.0f;         // 目标扭矩 (N·m)
    
    // 当前状态
    float current_speed_ = 0.0f;         // 当前速度 (m/s)
    float current_acceleration_ = 0.0f;  // 当前加速度 (m/s²)
    float current_steering_angle_ = 0.0f; // 当前转向角 (rad)
    
    // CAN接口配置
    std::string can_channel_ = "can0";   // CAN通道名称
    
    // 组件
    std::unique_ptr<data_input::StateEstimator> estimator_;
    std::unique_ptr<vehicle_control::VehicleModel> vehicle_model_;
    std::unique_ptr<vehicle_control::TorqueAllocator> allocator_;
    std::unique_ptr<vehicle_control::TorqueSmoother> smoother_;
    std::vector<std::unique_ptr<vehicle_control::TireModel>> tire_models_;
    std::unique_ptr<vehicle_control::CANBusInterface> can_interface_;
    
    // 输出状态
    std::vector<vehicle_control::WheelState> wheel_states_;
    std::vector<float> wheel_torques_;
};

// ChassisController 类方法实现
ChassisController::ChassisController() 
    : impl_(std::make_unique<ChassisControlImpl>()) {
}

ChassisController::~ChassisController() = default;

bool ChassisController::loadConfig(const std::string& config_file) {
    return impl_->loadConfig(config_file);
}

bool ChassisController::initialize(float vehicle_mass,
                                  float wheel_radius,
                                  float wheel_base,
                                  float track_width,
                                  float max_steering_angle,
                                  float max_torque,
                                  const std::string& can_channel) {
    return impl_->initialize(vehicle_mass, wheel_radius, wheel_base, 
                             track_width, max_steering_angle, max_torque, 
                             can_channel);
}

void ChassisController::setControlMode(ControlMode mode) {
    impl_->setControlMode(mode);
}

void ChassisController::setTargetSpeed(float speed) {
    impl_->setTargetSpeed(speed);
}

void ChassisController::setTargetAcceleration(float acceleration) {
    impl_->setTargetAcceleration(acceleration);
}

void ChassisController::setSteeringAngle(float angle) {
    impl_->setSteeringAngle(angle);
}

void ChassisController::setAllocatorType(AllocatorType type) {
    impl_->setAllocatorType(type);
}

void ChassisController::setSmootherType(SmootherType type) {
    impl_->setSmootherType(type);
}

void ChassisController::setPIDParameters(float kp, float ki, float kd) {
    impl_->setPIDParameters(kp, ki, kd);
}

void ChassisController::setLowPassFilterAlpha(float alpha) {
    impl_->setLowPassFilterAlpha(alpha);
}

void ChassisController::setMovingAverageWindowSize(size_t window_size) {
    impl_->setMovingAverageWindowSize(window_size);
}

void ChassisController::setRateLimiter(float max_rate) {
    impl_->setRateLimiter(max_rate);
}

bool ChassisController::updateVehicleState(float speed, float acceleration, float steering_angle) {
    return impl_->updateVehicleState(speed, acceleration, steering_angle);
}

bool ChassisController::control(float target_speed, float target_acceleration, float target_steering_angle) {
    return impl_->control(target_speed, target_acceleration, target_steering_angle);
}

bool ChassisController::emergencyStop() {
    return impl_->emergencyStop();
}

std::vector<float> ChassisController::getWheelTorques() const {
    return impl_->getWheelTorques();
}

float ChassisController::getCurrentSpeed() const {
    return impl_->getCurrentSpeed();
}

float ChassisController::getCurrentAcceleration() const {
    return impl_->getCurrentAcceleration();
}

float ChassisController::getCurrentSteeringAngle() const {
    return impl_->getCurrentSteeringAngle();
}

bool ChassisController::sendDiagnostics() {
    return impl_->sendDiagnostics();
}

void ChassisController::setTireFrictionCoefficient(float mu, int wheel_index) {
    impl_->setTireFrictionCoefficient(mu, wheel_index);
}

void ChassisController::setTireLoad(float load, int wheel_index) {
    impl_->setTireLoad(load, wheel_index);
}

void ChassisController::setTireSlipThreshold(float threshold, int wheel_index) {
    impl_->setTireSlipThreshold(threshold, wheel_index);
}

bool ChassisController::isTireSliding(int wheel_index) const {
    return impl_->isTireSliding(wheel_index);
}

float ChassisController::getTireSlipRatio(int wheel_index) const {
    return impl_->getTireSlipRatio(wheel_index);
}

float ChassisController::getTireLongitudinalForce(int wheel_index) const {
    return impl_->getTireLongitudinalForce(wheel_index);
}

float ChassisController::getTireLoad(int wheel_index) const {
    return impl_->getTireLoad(wheel_index);
}

void ChassisController::setTireRadius(float radius, int wheel_index) {
    impl_->setTireRadius(radius, wheel_index);
}

void ChassisController::setTireWidth(float width, int wheel_index) {
    impl_->setTireWidth(width, wheel_index);
}

void ChassisController::setTireStiffness(float stiffness, int wheel_index) {
    impl_->setTireStiffness(stiffness, wheel_index);
}

void ChassisController::setTireDamping(float damping, int wheel_index) {
    impl_->setTireDamping(damping, wheel_index);
}

void ChassisController::setTireCorneringStiffness(float cornering_stiffness, int wheel_index) {
    impl_->setTireCorneringStiffness(cornering_stiffness, wheel_index);
}

float ChassisController::getTireLateralForce(int wheel_index) const {
    return impl_->getTireLateralForce(wheel_index);
}

float ChassisController::getTireContactPatchArea(int wheel_index) const {
    return impl_->getTireContactPatchArea(wheel_index);
}

float ChassisController::getTireVerticalDisplacement(int wheel_index) const {
    return impl_->getTireVerticalDisplacement(wheel_index);
}

float ChassisController::getTireRadius(int wheel_index) const {
    return impl_->getTireRadius(wheel_index);
}

float ChassisController::getTireWidth(int wheel_index) const {
    return impl_->getTireWidth(wheel_index);
}

void ChassisController::update() {
    impl_->update();
}

} // namespace chassis_control 