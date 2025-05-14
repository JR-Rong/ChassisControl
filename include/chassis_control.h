/**
 * @file chassis_control.h
 * @brief 底盘控制系统的高级接口头文件
 * 
 * 本文件定义了底盘控制系统的简单高级接口，封装了底层的数据输入、车辆控制和数据输出模块，
 * 为用户提供易于使用的API，简化底盘控制系统的使用。
 */

#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <string>
#include <vector>
#include <memory>

namespace chassis_control {

// 前向声明内部类，隐藏实现细节
class ChassisControlImpl;

/**
 * @brief 控制模式枚举
 */
enum ControlMode {
    SPEED_CONTROL,  ///< 速度控制模式
    TORQUE_CONTROL, ///< 扭矩控制模式
    POSITION_CONTROL ///< 位置控制模式
};

/**
 * @brief 分配器类型枚举
 */
enum AllocatorType {
    EQUAL_DISTRIBUTION,  ///< 均等分配
    PROPORTIONAL_DISTRIBUTION, ///< 按比例分配
    OPTIMAL_DISTRIBUTION ///< 最优分配
};

/**
 * @brief 平滑器类型枚举
 */
enum SmootherType {
    LOW_PASS_FILTER, ///< 低通滤波
    MOVING_AVERAGE,  ///< 移动平均
    PID_CONTROL,     ///< PID控制
    RATE_LIMITER     ///< 变化率限制
};

/**
 * @brief 底盘控制器类
 * 
 * 提供简单的API用于控制底盘，封装了底层的各种模块
 */
class ChassisController {
public:
    /**
     * @brief 构造函数
     */
    ChassisController();
    
    /**
     * @brief 析构函数
     */
    ~ChassisController();
    
    /**
     * @brief 从配置文件中加载配置
     * 
     * @param config_file 配置文件路径
     * @return bool 是否成功加载
     */
    bool loadConfig(const std::string& config_file);
    
    /**
     * @brief 初始化控制器
     * 
     * @param vehicle_mass 车辆质量(kg)
     * @param wheel_radius 车轮半径(m)
     * @param wheel_base 轴距(m)
     * @param track_width 轮距(m)
     * @param max_steering_angle 最大转向角(rad)
     * @param max_torque 最大扭矩(Nm)
     * @param can_channel CAN通道
     * @return bool 是否初始化成功
     */
    bool initialize(float vehicle_mass,
                    float wheel_radius,
                    float wheel_base,
                    float track_width,
                    float max_steering_angle,
                    float max_torque,
                    const std::string& can_channel);
    
    /**
     * @brief 设置控制模式
     * @param mode 控制模式 (扭矩/速度/加速度)
     */
    void setControlMode(ControlMode mode);
    
    /**
     * @brief 设置目标速度
     * @param speed 目标速度 (m/s)
     */
    void setTargetSpeed(float speed);
    
    /**
     * @brief 设置目标加速度
     * @param acceleration 目标加速度 (m/s²)
     */
    void setTargetAcceleration(float acceleration);
    
    /**
     * @brief 设置转向角
     * @param angle 转向角 (度)
     */
    void setSteeringAngle(float angle);
    
    /**
     * @brief 设置分配器类型
     * 
     * @param type 分配器类型
     */
    void setAllocatorType(AllocatorType type);
    
    /**
     * @brief 设置平滑器类型
     * 
     * @param type 平滑器类型
     */
    void setSmootherType(SmootherType type);
    
    /**
     * @brief 设置PID参数
     * 
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     */
    void setPIDParameters(float kp, float ki, float kd);
    
    /**
     * @brief 设置低通滤波参数
     * 
     * @param alpha 滤波系数 (0-1)
     */
    void setLowPassFilterAlpha(float alpha);
    
    /**
     * @brief 设置移动平均窗口大小
     * 
     * @param window_size 窗口大小
     */
    void setMovingAverageWindowSize(size_t window_size);
    
    /**
     * @brief 设置变化率限制
     * 
     * @param max_rate 最大变化率
     */
    void setRateLimiter(float max_rate);

    /**
     * @brief 设置轮胎摩擦系数
     * 
     * @param mu 摩擦系数 (0.0-1.0)
     * @param wheel_index 车轮索引 (0-3)，-1表示所有车轮
     */
    void setTireFrictionCoefficient(float mu, int wheel_index = -1);
    
    /**
     * @brief 设置轮胎加载力
     * 
     * @param load 垂直加载力 (N)
     * @param wheel_index 车轮索引 (0-3)，-1表示所有车轮
     */
    void setTireLoad(float load, int wheel_index = -1);
    
    /**
     * @brief 设置轮胎滑移比阈值
     * 
     * @param threshold 滑移比阈值，超过此值认为轮胎滑动
     * @param wheel_index 车轮索引 (0-3)，-1表示所有车轮
     */
    void setTireSlipThreshold(float threshold, int wheel_index = -1);
    
    /**
     * @brief 设置轮胎半径
     * 
     * @param radius 轮胎半径 (m)
     * @param wheel_index 车轮索引 (0-3)，-1表示所有车轮
     */
    void setTireRadius(float radius, int wheel_index = -1);
    
    /**
     * @brief 设置轮胎宽度
     * 
     * @param width 轮胎宽度 (m)
     * @param wheel_index 车轮索引 (0-3)，-1表示所有车轮
     */
    void setTireWidth(float width, int wheel_index = -1);
    
    /**
     * @brief 设置轮胎刚度
     * 
     * @param stiffness 轮胎刚度 (N/m)
     * @param wheel_index 车轮索引 (0-3)，-1表示所有车轮
     */
    void setTireStiffness(float stiffness, int wheel_index = -1);
    
    /**
     * @brief 设置轮胎阻尼系数
     * 
     * @param damping 阻尼系数 (Ns/m)
     * @param wheel_index 车轮索引 (0-3)，-1表示所有车轮
     */
    void setTireDamping(float damping, int wheel_index = -1);
    
    /**
     * @brief 设置轮胎侧偏刚度
     * 
     * @param cornering_stiffness 侧偏刚度 (N/rad)
     * @param wheel_index 车轮索引 (0-3)，-1表示所有车轮
     */
    void setTireCorneringStiffness(float cornering_stiffness, int wheel_index = -1);
    
    /**
     * @brief 获取轮胎滑动状态
     * 
     * @param wheel_index 车轮索引 (0-3)
     * @return bool 轮胎是否处于滑动状态
     */
    bool isTireSliding(int wheel_index) const;
    
    /**
     * @brief 获取轮胎滑移比
     * 
     * @param wheel_index 车轮索引 (0-3)
     * @return float 当前滑移比
     */
    float getTireSlipRatio(int wheel_index) const;
    
    /**
     * @brief 获取轮胎纵向力
     * 
     * @param wheel_index 车轮索引 (0-3)
     * @return float 纵向力 (N)
     */
    float getTireLongitudinalForce(int wheel_index) const;
    
    /**
     * @brief 获取轮胎横向力
     * 
     * @param wheel_index 车轮索引 (0-3)
     * @return float 横向力 (N)
     */
    float getTireLateralForce(int wheel_index) const;
    
    /**
     * @brief 获取轮胎接触面积
     * 
     * @param wheel_index 车轮索引 (0-3)
     * @return float 接触面积 (m^2)
     */
    float getTireContactPatchArea(int wheel_index) const;
    
    /**
     * @brief 获取轮胎垂向位移
     * 
     * @param wheel_index 车轮索引 (0-3)
     * @return float 垂向位移 (m)
     */
    float getTireVerticalDisplacement(int wheel_index) const;
    
    /**
     * @brief 获取轮胎半径
     * 
     * @param wheel_index 车轮索引 (0-3)
     * @return float 轮胎半径 (m)
     */
    float getTireRadius(int wheel_index = 0) const;
    
    /**
     * @brief 获取轮胎宽度
     * 
     * @param wheel_index 车轮索引 (0-3)
     * @return float 轮胎宽度 (m)
     */
    float getTireWidth(int wheel_index) const;
    
    /**
     * @brief 更新车辆状态
     * 
     * @param speed 当前速度(m/s)
     * @param acceleration 当前加速度(m/s^2)
     * @param steering_angle 当前转向角(rad)
     * @return bool 更新是否成功
     */
    bool updateVehicleState(float speed, float acceleration, float steering_angle);
    
    /**
     * @brief 更新控制器状态
     * 
     * 这个方法应该在每个控制周期调用一次。它会根据当前设置的控制模式、
     * 目标速度、加速度和转向角，计算并分配每个车轮的扭矩，然后通过CAN总线
     * 发送控制命令。
     */
    void update();
    
    /**
     * @brief 控制指令
     * 
     * @param target_speed 目标速度(m/s)
     * @param target_acceleration 目标加速度(m/s^2)
     * @param target_steering_angle 目标转向角(rad)
     * @return bool 控制是否成功
     */
    bool control(float target_speed, float target_acceleration, float target_steering_angle);
    
    /**
     * @brief 紧急停止
     * 
     * @return bool 是否成功执行
     */
    bool emergencyStop();
    
    /**
     * @brief 获取当前车轮扭矩
     * 
     * @return std::vector<float> 四个车轮的扭矩(Nm)
     */
    std::vector<float> getWheelTorques() const;
    
    /**
     * @brief 获取当前车速
     * 
     * @return float 车速(m/s)
     */
    float getCurrentSpeed() const;
    
    /**
     * @brief 获取当前加速度
     * 
     * @return float 加速度(m/s^2)
     */
    float getCurrentAcceleration() const;
    
    /**
     * @brief 获取当前转向角
     * 
     * @return float 转向角(rad)
     */
    float getCurrentSteeringAngle() const;
    
    /**
     * @brief 发送诊断信息
     * 
     * @return bool 是否成功发送
     */
    bool sendDiagnostics();
    
    /**
     * @brief 获取指定轮胎的宽度
     * @param wheel_index 轮胎索引 (0-3), -1代表不适用
     * @return 轮胎宽度 (m)
     */
    float getTireWidth(int wheel_index = 0) const;

    /**
     * @brief 获取指定轮胎的垂直位移
     * @param wheel_index 轮胎索引 (0-3), -1代表不适用
     * @return 轮胎垂直位移 (m)
     */
    float getTireVerticalDisplacement(int wheel_index = 0) const;

    /**
     * @brief 获取指定轮胎的接触面积
     * @param wheel_index 轮胎索引 (0-3), -1代表不适用
     * @return 轮胎接触面积 (m²)
     */
    float getTireContactPatchArea(int wheel_index = 0) const;

    /**
     * @brief 获取指定轮胎的侧向力
     * @param wheel_index 轮胎索引 (0-3), -1代表不适用
     * @return 轮胎侧向力 (N)
     */
    float getTireLateralForce(int wheel_index = 0) const;

    /**
     * @brief 获取指定轮胎的半径
     * @param wheel_index 轮胎索引 (0-3)
     * @return 轮胎半径 (m)
     */
    float getTireRadius(int wheel_index = 0) const;

    /**
     * @brief 设置指定轮胎的半径
     * @param radius 轮胎半径 (m)
     * @param wheel_index 轮胎索引 (0-3), -1代表设置所有轮胎
     */
    void setTireRadius(float radius, int wheel_index = -1);

    /**
     * @brief 设置指定轮胎的宽度
     * @param width 轮胎宽度 (m)
     * @param wheel_index 轮胎索引 (0-3), -1代表设置所有轮胎
     */
    void setTireWidth(float width, int wheel_index = -1);

    /**
     * @brief 设置指定轮胎的刚度
     * @param stiffness 轮胎刚度 (N/m)
     * @param wheel_index 轮胎索引 (0-3), -1代表设置所有轮胎
     */
    void setTireStiffness(float stiffness, int wheel_index = -1);

    /**
     * @brief 设置指定轮胎的阻尼系数
     * @param damping 轮胎阻尼系数 (N·s/m)
     * @param wheel_index 轮胎索引 (0-3), -1代表设置所有轮胎
     */
    void setTireDamping(float damping, int wheel_index = -1);

    /**
     * @brief 设置指定轮胎的侧偏刚度
     * @param cornering_stiffness 轮胎侧偏刚度 (N/rad)
     * @param wheel_index 轮胎索引 (0-3), -1代表设置所有轮胎
     */
    void setTireCorneringStiffness(float cornering_stiffness, int wheel_index = -1);

    /**
     * @brief 获取指定轮胎是否处于滑移状态
     * @param wheel_index 轮胎索引 (0-3)
     * @return 如果轮胎处于滑移状态，返回true；否则返回false
     */
    bool getTireSliding(int wheel_index = 0) const;
    
    /**
     * @brief 获取指定轮胎的滑移比
     * @param wheel_index 轮胎索引 (0-3)
     * @return 轮胎滑移比 (无量纲)
     */
    float getTireSlipRatio(int wheel_index = 0) const;
    
    /**
     * @brief 获取指定轮胎的纵向力
     * @param wheel_index 轮胎索引 (0-3)
     * @return 轮胎纵向力 (N)
     */
    float getTireLongitudinalForce(int wheel_index = 0) const;
    
    /**
     * @brief 获取指定轮胎的载荷（垂直力）
     * @param wheel_index 轮胎索引 (0-3)
     * @return 轮胎垂直载荷 (N)
     */
    float getTireLoad(int wheel_index = 0) const;

private:
    /// 底盘控制实现类，使用PIMPL模式隐藏实现细节
    std::unique_ptr<ChassisControlImpl> impl_;
};

} // namespace chassis_control

#endif // CHASSIS_CONTROL_H 