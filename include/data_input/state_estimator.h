/**
 * @file state_estimator.h
 * @brief 状态估计器类
 */

#ifndef CHASSIS_CONTROL_STATE_ESTIMATOR_H
#define CHASSIS_CONTROL_STATE_ESTIMATOR_H

#include <vector>
#include <deque>
#include <cstdint>

namespace chassis_control {
namespace data_input {

/**
 * @class StateEstimator
 * @brief 车辆状态估计器，提供传感器数据滤波和状态估计功能
 */
class StateEstimator {
public:
    /**
     * @brief 默认构造函数
     */
    StateEstimator();
    
    /**
     * @brief 带参数的构造函数
     * @param filter_alpha 滤波系数(0-1)，值越小滤波效果越强
     * @param window_size 滑动窗口大小
     */
    StateEstimator(float filter_alpha, size_t window_size = 10);
    
    /**
     * @brief 析构函数
     */
    ~StateEstimator();
    
    /**
     * @brief 更新车辆状态
     * @param speed 当前速度(m/s)
     * @param accel 当前加速度(m/s^2)
     */
    void update(float speed, float accel);
    
    /**
     * @brief 更新车辆状态(包含姿态数据)
     * @param speed 当前速度(m/s)
     * @param accel 当前加速度(m/s^2)
     * @param attitude 当前姿态数据[横摇角,俯仰角,偏航角,横摇角速度,俯仰角速度,偏航角速度]
     */
    void update(float speed, float accel, const std::vector<float>& attitude);
    
    /**
     * @brief 获取滤波后的速度
     * @return 滤波后的速度(m/s)
     */
    float getFilteredSpeed() const;
    
    /**
     * @brief 获取滤波后的加速度
     * @return 滤波后的加速度(m/s^2)
     */
    float getFilteredAcceleration() const;
    
    /**
     * @brief 获取估计的姿态
     * @return 估计的姿态数组[横摇角,俯仰角,偏航角,横摇角速度,俯仰角速度,偏航角速度]
     */
    std::vector<float> getEstimatedAttitude() const;
    
    /**
     * @brief 设置滤波系数
     * @param alpha 滤波系数(0-1)，值越小滤波效果越强
     */
    void setFilterAlpha(float alpha);
    
    /**
     * @brief 设置滑动窗口大小
     * @param size 窗口大小
     */
    void setWindowSize(size_t size);
    
    /**
     * @brief 重置状态估计器
     */
    void reset();
    
    /**
     * @brief 打印当前状态(用于调试)
     */
    void printState() const;

private:
    // 滤波后的状态
    float smoothSpeed = 0.0f;       ///< 滤波后的速度(m/s)
    float smoothAccel = 0.0f;       ///< 滤波后的加速度(m/s^2)
    float alpha = 0.1f;             ///< 滤波系数(0-1)，值越小滤波效果越强
    
    // 估计的姿态
    std::vector<float> estimatedAttitude = std::vector<float>(6, 0.0f);  ///< 估计的姿态
    
    // 历史数据窗口
    size_t windowSize = 10;                     ///< 滑动窗口大小
    std::deque<float> speedWindow;              ///< 速度历史窗口
    std::deque<float> accelWindow;              ///< 加速度历史窗口
    std::vector<std::deque<float>> attitudeWindows;  ///< 姿态历史窗口
    
    // 上次更新时间戳
    uint64_t lastUpdateTime = 0;                ///< 上次更新时间戳(微秒)
    
    /**
     * @brief 获取当前时间戳
     * @return 当前时间戳(微秒)
     */
    uint64_t getCurrentTimestamp() const;
    
    /**
     * @brief 应用一阶低通滤波
     * @param current 当前值
     * @param previous 上一次滤波值
     * @return 滤波后的值
     */
    float applyLowPassFilter(float current, float previous) const;
    
    /**
     * @brief 应用移动平均滤波
     * @param window 数据窗口
     * @return 滤波后的值
     */
    float applyMovingAverageFilter(const std::deque<float>& window) const;
    
    /**
     * @brief 更新滑动窗口
     * @param window 数据窗口
     * @param value 新值
     */
    void updateWindow(std::deque<float>& window, float value);
};

} // namespace data_input
} // namespace chassis_control

#endif // CHASSIS_CONTROL_STATE_ESTIMATOR_H 