/**
 * @file torque_smoother.h
 * @brief 扭矩平滑处理类
 */

#ifndef CHASSIS_CONTROL_TORQUE_SMOOTHER_H
#define CHASSIS_CONTROL_TORQUE_SMOOTHER_H

#include <vector>
#include <deque>
#include <cstdint>
#include <memory>

namespace chassis_control {
namespace vehicle_control {

/**
 * @enum SmootherType
 * @brief 平滑器类型枚举
 */
enum class SmootherType {
    LOW_PASS_FILTER,      ///< 低通滤波器
    MOVING_AVERAGE,       ///< 移动平均滤波
    PID_CONTROLLER,       ///< PID控制器
    RATE_LIMITER          ///< 变化率限制器
};

/**
 * @struct PIDParams
 * @brief PID控制器参数
 */
struct PIDParams {
    float kp;             ///< 比例增益
    float ki;             ///< 积分增益
    float kd;             ///< 微分增益
    float outputMin;      ///< 输出下限
    float outputMax;      ///< 输出上限
    
    PIDParams() 
        : kp(1.0f), ki(0.1f), kd(0.05f), 
          outputMin(-1000.0f), outputMax(1000.0f) {}
    
    PIDParams(float p, float i, float d, float min, float max) 
        : kp(p), ki(i), kd(d), outputMin(min), outputMax(max) {}
};

/**
 * @class TorqueSmoother
 * @brief 扭矩平滑处理类，对计算出的扭矩进行平滑处理，以提高控制品质
 */
class TorqueSmoother {
public:
    /**
     * @brief 默认构造函数
     */
    TorqueSmoother();
    
    /**
     * @brief 带参数的构造函数
     * @param smoother_type 平滑器类型
     * @param filter_alpha 滤波系数(0-1)，仅用于低通滤波
     * @param window_size 窗口大小，仅用于移动平均滤波
     */
    TorqueSmoother(SmootherType smoother_type, float filter_alpha = 0.1f, size_t window_size = 5);
    
    /**
     * @brief 析构函数
     */
    ~TorqueSmoother();
    
    /**
     * @brief 平滑单个扭矩值
     * @param raw_torque 原始扭矩值(Nm)
     * @param wheel_index 车轮索引，用于跟踪不同车轮的历史数据
     * @return 平滑后的扭矩值(Nm)
     */
    float smoothTorque(float raw_torque, size_t wheel_index);
    
    /**
     * @brief 平滑多个扭矩值
     * @param raw_torques 原始扭矩值数组(Nm)
     * @return 平滑后的扭矩值数组(Nm)
     */
    std::vector<float> smoothTorques(const std::vector<float>& raw_torques);
    
    /**
     * @brief 设置平滑器类型
     * @param type 平滑器类型
     */
    void setSmootherType(SmootherType type);
    
    /**
     * @brief 设置低通滤波系数
     * @param alpha 滤波系数(0-1)，值越小滤波效果越强
     */
    void setFilterAlpha(float alpha);
    
    /**
     * @brief 设置移动平均窗口大小
     * @param size 窗口大小
     */
    void setWindowSize(size_t size);
    
    /**
     * @brief 设置PID参数
     * @param params PID参数
     */
    void setPIDParams(const PIDParams& params);
    
    /**
     * @brief 设置变化率限制
     * @param max_rate 最大变化率(Nm/s)
     * @param sample_time 采样时间(s)
     */
    void setRateLimit(float max_rate, float sample_time = 0.01f);
    
    /**
     * @brief 重置平滑器状态
     */
    void reset();
    
private:
    /**
     * @brief 应用低通滤波
     * @param current 当前值
     * @param previous 上一次滤波值
     * @return 滤波后的值
     */
    float applyLowPassFilter(float current, float previous) const;
    
    /**
     * @brief 应用移动平均滤波
     * @param value 当前值
     * @param window 历史数据窗口
     * @return 滤波后的值
     */
    float applyMovingAverage(float value, std::deque<float>& window);
    
    /**
     * @brief 应用PID控制
     * @param setpoint 目标值
     * @param current 当前值
     * @param wheel_index 车轮索引
     * @return PID控制输出
     */
    float applyPIDControl(float setpoint, float current, size_t wheel_index);
    
    /**
     * @brief 应用变化率限制
     * @param current 当前目标值
     * @param previous 上一次输出值
     * @return 限制变化率后的值
     */
    float applyRateLimiter(float current, float previous) const;
    
    /**
     * @brief 获取当前时间戳(微秒)
     * @return 当前时间戳
     */
    uint64_t getCurrentTimestamp() const;

private:
    SmootherType smootherType_;                 ///< 平滑器类型
    float alpha_;                               ///< 低通滤波系数
    size_t windowSize_;                         ///< 移动平均窗口大小
    std::vector<std::deque<float>> windows_;    ///< 各车轮历史数据窗口
    std::vector<float> lastOutputs_;            ///< 各车轮上次输出值
    
    // PID控制相关
    PIDParams pidParams_;                       ///< PID参数
    std::vector<float> pidErrors_;              ///< 各车轮误差
    std::vector<float> pidIntegrals_;           ///< 各车轮积分项
    std::vector<float> pidDerivatives_;         ///< 各车轮微分项
    std::vector<uint64_t> lastUpdateTimes_;     ///< 各车轮上次更新时间
    
    // 变化率限制相关
    float maxRateOfChange_;                     ///< 最大变化率(Nm/s)
    float sampleTime_;                          ///< 采样时间(s)
};

} // namespace vehicle_control
} // namespace chassis_control

#endif // CHASSIS_CONTROL_TORQUE_SMOOTHER_H 