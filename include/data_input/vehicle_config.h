/**
 * @file vehicle_config.h
 * @brief 整车固定参数配置读取类
 */

#ifndef CHASSIS_CONTROL_VEHICLE_CONFIG_H
#define CHASSIS_CONTROL_VEHICLE_CONFIG_H

#include <string>
#include <vector>
#include <utility>  // for std::pair

namespace chassis_control {
namespace data_input {

/**
 * @class VehicleConfig
 * @brief 车辆配置类，负责读取和管理车辆固定参数
 */
class VehicleConfig {
public:
    /**
     * @brief 默认构造函数
     */
    VehicleConfig();
    
    /**
     * @brief 析构函数
     */
    ~VehicleConfig();
    
    /**
     * @brief 从YAML文件加载配置
     * @param yamlPath YAML配置文件路径
     * @return 是否加载成功
     */
    bool loadFromFile(const std::string& yamlPath);
    
    /**
     * @brief 获取车辆质量
     * @return 车辆质量(kg)
     */
    float getMass() const;
    
    /**
     * @brief 获取车轮半径
     * @return 车轮半径(m)
     */
    float getWheelRadius() const;
    
    /**
     * @brief 获取轴距
     * @return 轴距(m)
     */
    float getWheelBase() const;
    
    /**
     * @brief 获取轮距
     * @return 轮距(m)
     */
    float getTrackWidth() const;
    
    /**
     * @brief 获取质心位置
     * @return 质心位置(x, y)，单位m
     */
    std::pair<float, float> getCoGPosition() const;
    
    /**
     * @brief 获取所有车轮位置
     * @return 车轮位置数组，每个元素为(x, y)，单位m
     */
    const std::vector<std::pair<float, float>>& getWheelPositions() const;
    
    /**
     * @brief 获取车轮参数
     * @return 车轮参数
     */
    float getWheelInfor() const;
    
    /**
     * @brief 获取最大转向角
     * @return 最大转向角(rad)
     */
    float getMaxSteeringAngle() const;
    
    /**
     * @brief 获取最大扭矩
     * @return 最大扭矩(Nm)
     */
    float getMaxTorque() const;
    
    /**
     * @brief 获取最大速度
     * @return 最大速度(m/s)
     */
    float getMaxVelocity() const;
    
    /**
     * @brief 获取最大加速度
     * @return 最大加速度(m/s^2)
     */
    float getMaxAcceleration() const;
    
    /**
     * @brief 获取最大减速度
     * @return 最大减速度(m/s^2)
     */
    float getMaxDeceleration() const;
    
    /**
     * @brief 获取齿轮比
     * @return 齿轮比
     */
    float getGearRatio() const;
    
    /**
     * @brief 打印车辆配置信息（用于调试）
     */
    void printConfig() const;

private:
    float mass = 1000.0f;                             ///< 车辆质量(kg)
    float wheelRadius = 0.3f;                         ///< 车轮半径(m)
    float wheelBase = 2.0f;                           ///< 轴距(m)
    float trackWidth = 1.5f;                          ///< 轮距(m)
    std::pair<float, float> cogPosition = {0.0f, 0.0f}; ///< 质心位置(x, y)，单位m
    std::vector<std::pair<float, float>> wheelPositions; ///< 车轮位置数组，每个元素为(x, y)，单位m
    
    // 附加参数
    float maxSteeringAngle = 0.5f;                    ///< 最大转向角(rad)
    float maxTorque = 500.0f;                         ///< 最大扭矩(Nm)
    float maxVelocity = 20.0f;                        ///< 最大速度(m/s)
    float maxAcceleration = 3.0f;                     ///< 最大加速度(m/s^2)
    float maxDeceleration = 5.0f;                     ///< 最大减速度(m/s^2)
    float gearRatio = 10.0f;                          ///< 齿轮比
    float wheelInfor = 0.0f;                          ///< 车轮参数
    int wheelCount = 4;                               ///< 车轮数量
    
    /**
     * @brief 验证车辆参数有效性
     * @return 是否有效
     */
    bool validateParameters() const;
    
    /**
     * @brief 初始化默认车轮位置
     */
    void initializeDefaultWheelPositions();
};

} // namespace data_input
} // namespace chassis_control

#endif // CHASSIS_CONTROL_VEHICLE_CONFIG_H 