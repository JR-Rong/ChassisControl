/**
 * @file vehicle_model.h
 * @brief 车辆模型类的头文件
 * 
 * 本文件定义了车辆模型类，用于模拟车辆运动学和动力学行为。
 */

#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include <vector>
#include <functional>
#include <utility>

namespace vehicle_control {

/**
 * @brief 方向盘转角到车轮转角的映射函数类型
 */
using SteeringFunction = std::function<std::vector<float>(float, float, float, float)>;

/**
 * @brief 车轮状态结构体
 */
struct WheelState {
    float steeringAngle;         ///< 转向角(rad)
    float angularVelocity;       ///< 角速度(rad/s)
    float slipRatio;             ///< 滑移比
    float normalForce;           ///< 法向力(N)
    float longitudinalForce;     ///< 纵向力(N)
    float lateralForce;          ///< 横向力(N)
    
    /**
     * @brief 默认构造函数
     */
    WheelState() 
        : steeringAngle(0.0f),
          angularVelocity(0.0f),
          slipRatio(0.0f),
          normalForce(0.0f),
          longitudinalForce(0.0f),
          lateralForce(0.0f) {}
};

/**
 * @brief 车辆模型类
 * 
 * 模拟车辆的运动学特性，计算车轮状态
 */
class VehicleModel {
public:
    /**
     * @brief 驱动模式枚举
     */
    enum class DriveMode {
        FRONT_WHEEL_DRIVE,     ///< 前轮驱动
        REAR_WHEEL_DRIVE,      ///< 后轮驱动
        ALL_WHEEL_DRIVE        ///< 四轮驱动
    };
    
    /**
     * @brief 构造函数
     * 
     * @param wheel_positions 车轮位置(x,y)向量，相对于车辆中心(m)
     * @param wheel_radius 车轮半径(m)
     * @param wheel_base 轴距(m)
     * @param track_width 轮距(m)
     * @param max_steering_angle 最大转向角(rad)
     */
    VehicleModel(const std::vector<std::pair<float, float>>& wheel_positions,
                float wheel_radius,
                float wheel_base,
                float track_width,
                float max_steering_angle);
    
    /**
     * @brief 析构函数
     */
    ~VehicleModel();
    
    /**
     * @brief 计算车轮状态
     * 
     * @param velocity 车辆速度(m/s)
     * @param acceleration 车辆加速度(m/s^2)
     * @param steering_angle 转向角(rad)
     * @param steering_function 转向模型函数
     * @return std::vector<WheelState> 各轮状态
     */
    std::vector<WheelState> computeWheelStates(
        float velocity,
        float acceleration,
        float steering_angle,
        const SteeringFunction& steering_function);
    
    /**
     * @brief 设置驱动模式
     * 
     * @param mode 驱动模式
     */
    void setDriveMode(DriveMode mode);
    
    /**
     * @brief 获取车轮数量
     * 
     * @return size_t 车轮数量
     */
    size_t getWheelCount() const;
    
    /**
     * @brief 设置轴距
     * 
     * @param wheel_base 轴距(m)
     */
    void setWheelBase(float wheel_base);
    
    /**
     * @brief 获取轴距
     * 
     * @return float 轴距(m)
     */
    float getWheelBase() const;
    
    /**
     * @brief 设置轮距
     * 
     * @param track_width 轮距(m)
     */
    void setTrackWidth(float track_width);
    
    /**
     * @brief 获取轮距
     * 
     * @return float 轮距(m)
     */
    float getTrackWidth() const;
    
    /**
     * @brief 获取默认的阿克曼转向函数
     * 
     * @return SteeringFunction 阿克曼转向函数
     */
    SteeringFunction getDefaultAckermannFunction() const;
    
private:
    // 车轮位置
    std::vector<std::pair<float, float>> wheel_positions_;
    
    // 车轮数量
    size_t wheel_count_;
    
    // 车轮半径(m)
    float wheel_radius_;
    
    // 轴距(m)
    float wheel_base_;
    
    // 轮距(m)
    float track_width_;
    
    // 最大转向角(rad)
    float max_steering_angle_;
    
    // 驱动模式
    DriveMode drive_mode_;
    
    /**
     * @brief 阿克曼转向模型
     * 
     * @param steering_angle 方向盘转角(rad)
     * @param wheel_base 轴距(m)
     * @param track_width 轮距(m)
     * @param max_angle 最大转向角(rad)
     * @return std::vector<float> 四个车轮的转向角(rad)
     */
    std::vector<float> ackermannSteering(float steering_angle, float wheel_base, float track_width, float max_angle) const;
};

} // namespace vehicle_control

#endif // VEHICLE_MODEL_H 