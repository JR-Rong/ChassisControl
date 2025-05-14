/**
 * @file torque_allocator.h
 * @brief 扭矩分配器类
 */

#ifndef CHASSIS_CONTROL_TORQUE_ALLOCATOR_H
#define CHASSIS_CONTROL_TORQUE_ALLOCATOR_H

#include <vector>
#include "data_input/vehicle_config.h"
#include "vehicle_control/vehicle_model.h"

namespace chassis_control {
namespace vehicle_control {

/**
 * @enum AllocateMethod
 * @brief 扭矩分配方法枚举
 */
enum class AllocateMethod {
    EVEN,             ///< 均匀分配
    PROPORTIONAL,     ///< 按滑移率比例分配
    OPTIMAL,          ///< 优化分配(QP等方法)
    DYNAMIC_WEIGHT,   ///< 动态权重分配
    YAW_CONTROL       ///< 考虑横摆力矩的分配
};

/**
 * @struct AllocationParameters
 * @brief 扭矩分配参数结构体
 */
struct AllocationParameters {
    float maxTorquePerWheel;     ///< 每轮最大扭矩限制(Nm)
    float yawMomentTarget;       ///< 目标横摆力矩(Nm)
    float weightLongitudinal;    ///< 纵向加速权重
    float weightYaw;             ///< 横摆矩权重
    float weightMinimize;        ///< 总扭矩最小化权重
    
    AllocationParameters() :
        maxTorquePerWheel(1000.0f), 
        yawMomentTarget(0.0f),
        weightLongitudinal(1.0f),
        weightYaw(0.5f),
        weightMinimize(0.2f) {}
};

/**
 * @class TorqueAllocator
 * @brief 扭矩分配器，用于将总扭矩需求分配到各个车轮
 */
class TorqueAllocator {
public:
    /**
     * @brief 构造函数
     * @param config 车辆配置
     */
    TorqueAllocator(const data_input::VehicleConfig& config);
    
    /**
     * @brief 构造函数
     * @param vehicle_mass 车辆质量(kg)
     * @param wheel_radius 车轮半径(m)
     * @param wheel_positions 车轮位置
     */
    TorqueAllocator(float vehicle_mass, float wheel_radius, 
                    const std::vector<std::pair<float, float>>& wheel_positions);
                    
    /**
     * @brief 析构函数
     */
    ~TorqueAllocator();
    
    /**
     * @brief 设置车辆目标速度
     * @param target_vehicle_speed 目标车速(m/s)
     */
    void setVehicleSpeed(float target_vehicle_speed);
    
    /**
     * @brief 设置车轮目标速度(每个车轮可以不同)
     * @param target_wheel_speeds 各轮目标速度(m/s)
     */
    void setWheelSpeeds(const std::vector<float>& target_wheel_speeds);
    
    /**
     * @brief 设置转向角
     * @param steering_angle 转向角(rad)
     */
    void setSteeringAngle(float steering_angle);
    
    /**
     * @brief 设置扭矩分配方法
     * @param method 分配方法
     */
    void setAllocateMethod(AllocateMethod method);
    
    /**
     * @brief 设置分配参数
     * @param params 分配参数
     */
    void setParameters(const AllocationParameters& params);
    
    /**
     * @brief 分配扭矩
     * @param wheel_states 各轮状态
     * @param accel 目标加速度(m/s^2)
     * @return 各轮分配后的扭矩(Nm)
     */
    std::vector<float> allocate(const std::vector<WheelState>& wheel_states, float accel);
    
    /**
     * @brief 扭矩优化(二次规划等优化方法)
     * @param wheel_states 各轮状态
     * @param total_torque 需要分配的总扭矩
     * @return 优化后的扭矩分配
     */
    std::vector<float> optimizeTorque(const std::vector<WheelState>& wheel_states, float total_torque);
    
    /**
     * @brief 获取当前分配方法
     * @return 当前分配方法
     */
    AllocateMethod getAllocateMethod() const;
    
    /**
     * @brief 获取分配参数
     * @return 分配参数
     */
    AllocationParameters getParameters() const;

private:
    /**
     * @brief 初始化扭矩分配器
     * @param mass 车辆质量
     * @param radius 车轮半径
     */
    void initialize(float mass, float radius);

    /**
     * @brief 均匀分配扭矩
     * @param total_torque 需要分配的总扭矩
     * @param wheel_count 车轮数量
     * @return 均匀分配的扭矩
     */
    std::vector<float> evenAllocate(float total_torque, size_t wheel_count) const;
    
    /**
     * @brief 基于滑移率按比例分配扭矩
     * @param wheel_states 各轮状态
     * @param total_torque 需要分配的总扭矩
     * @return 按比例分配后的扭矩
     */
    std::vector<float> proportionalAllocate(const std::vector<WheelState>& wheel_states, float total_torque) const;
    
    /**
     * @brief 考虑横摆力矩的扭矩分配
     * @param wheel_states 各轮状态
     * @param total_torque 需要分配的总扭矩
     * @param yaw_moment_target 目标横摆力矩
     * @return 分配后的扭矩
     */
    std::vector<float> yawControlAllocate(const std::vector<WheelState>& wheel_states, 
                                        float total_torque, 
                                        float yaw_moment_target) const;
    
    /**
     * @brief 应用扭矩约束
     * @param torques 扭矩数组
     * @param max_torque 最大扭矩限制
     * @return 约束后的扭矩数组
     */
    std::vector<float> applyTorqueConstraints(const std::vector<float>& torques, float max_torque) const;

private:
    float vehicleMass;                                ///< 车辆质量(kg)
    float wheelRadius;                                ///< 车轮半径(m)
    std::vector<std::pair<float, float>> wheelPositions; ///< 车轮位置
    std::vector<float> targetWheelSpeeds;             ///< 各轮目标速度(m/s)
    float targetVehicleSpeed;                         ///< 目标车速(m/s)
    float steeringAngle;                              ///< 转向角(rad)
    AllocateMethod allocateMethod;                    ///< 扭矩分配方法
    AllocationParameters parameters;                  ///< 扭矩分配参数
};

} // namespace vehicle_control
} // namespace chassis_control

#endif // CHASSIS_CONTROL_TORQUE_ALLOCATOR_H 