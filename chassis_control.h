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
#include <map>
#include <array>

// 与轮胎特性相关的曲线
struct WheelCharacteristicsCurve {
    // 滑移率-纵向力
    std::vector<std::pair<double,double>> slip_vs_longitudinal_force;
    // 侧偏角-侧向力
    std::vector<std::pair<double,double>> slip_angle_vs_lateral_force;
    // 侧偏角-回正力矩曲线     
    std::vector<std::pair<double,double>> slip_angle_vs_aligning_torque;
    // 外倾角-侧向力曲线   
    std::vector<std::pair<double,double>> camber_vs_lateral_force;
    // 滚动阻力曲线         
    std::vector<std::pair<double,double>> rolling_resistance_curve; 
    // 轮胎刚度曲线       
    std::vector<std::pair<double,double>> wheel_stiffness_curve;     
};

//整车静态参数，当车型确定后将不再更改，只加载一次
struct StaticParam{
    //整车参数
    float max_steering_angle = 0.5f;                      // 最大转向角(rad)
    float max_torque = 500.0f;                            // 最大扭矩(Nm)
    float max_velocity = 20.0f;                           // 最大速度(m/s)
    float max_acceleration = 3.0f;                        // 最大加速度(m/s^2)
    float max_deceleration = 5.0f;                        // 最大减速度(m/s^2)
    float max_torquel_delta;                              // 扭矩变化最大幅值   
    int wheel_count = 4;                                  // 车轮数量
    // 轮胎特性曲线
    std::map<std::string, WheelCharacteristicsCurve > characteristics_curve;
};

//轮胎参数
struct WheelParam {
    //基本参数
    int wheel_col=-1;
    int wheel_row=-1; 
    //轮胎参数
    std::string feture_name;               //轮胎特性曲线名称
    float mu;                              // 静摩擦系数
    float slip_threshold;                  // 滑移阈值
    float wheel_width;                     // 轮胎宽度(m)           
    std::array<float,3> wheel_position;    // 车轮位置(x,y,z),单位m
    float wheel_radius;                    // 轮胎滚动半径(m)
    float slip_ratio;                      // 滑移率
    float wheel_pressure;                  //胎压
};

struct DynamicParam {
    float mass = 1000.0f;                                  // 车辆质量(kg)
    std::array<float,3> cog_position = {0.0f,0.0f,0.0f};   // 质心位置(x, y, z)，单位m
    std::vector<WheelParam> wheel_params;                  // 车轮参数数组
};

struct VehicleParam{
    StaticParam vechicle_static_param;
    DynamicParam  vechicle_dynamic_param;   
};

 //车轮状态
 struct WheelState {
    //基本参数
    int wheel_col=-1;
    int wheel_row=-1;
    std::array<float,2> steeringAngle;         // 转向角(rad)
    std::array<float,2> angularVelocity;       // 角速度(rad/s）
    bool is_sliding;                           // 是否处于滑动状态
    float vertical_displacement;               // 垂向位移(m)
};

//整车运行状态，在整车运行过程中需按照固定时间间隔或消息帧间隔进行状态更新
struct VehicleState{
    std::array<float,2> cur_velocity;                // 当前车辆速度(纵向，横向)
    std::array<float,2> cur_acceleration;            // 当前车辆加速度(纵向，横向)
    float cur_steering_angle;                        // 当前车辆转向角
    std::array<float,2> target_velocity;             // 目标车辆速度(纵向，横向)
    std::array<float,2> target_acceleration;         // 目标车辆加速度(纵向，横向)
    float target_steering_angle;                     // 目标车辆转向角
    std::array<float,3> target_position;             // 目标车辆位置
    std::vector<WheelState> tier_state;              // 车轮当前状态 
    std::array<float,3> vehicle_posture;             // 整车当前姿态
    std::array<float,3> vehicle_position;           // 整车当前位置
  };

//车辆控制模式
struct ControlMode{

};

//  /**
//  * @brief 从配置文件中加载配置
//  * @param config_file 配置文件路径
//  * @return bool 是否成功加载
//  */
// bool LoadConfig(const std::string& config_file);

/**
 * @brief 初始化
 * @param vehicle 整车参数
 * @param can_channel CAN通道
 * @return bool 是否初始化成功
 */
bool Initialize(const VehicleParam& vehicle, const std::string& can_channel);

/**
 * @brief 设置控制模式
 * @param mode 控制模式 (扭矩/速度/加速度)
 */
void SetControlMode(const ControlMode mode);

/**
 * @brief 更新车辆状态
 * @param vehicle_state 当前车辆运行状态
 * @return bool 更新是否成功
 */ 
bool UpdateVehicleState(const VehicleState& vehicle_state);

/**
 * @brief 更新整车参数
 * @param vehicle_dyanmic_param 待更新的整车参数
 */ 
void UpdateDynamicParam(const DynamicParam& vehicle_dynamic_param);

/**
 * @brief 紧急停止
 * @return bool 是否成功执行
 */
bool EmergencyStop();

bool SendCanMessage(const std::string& channel_name,int can_id);

#endif // CHASSIS_CONTROL_H 
