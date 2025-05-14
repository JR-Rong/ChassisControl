/**
 * @file tire_model.h
 * @brief 轮胎模型类的头文件
 * 
 * 本文件定义了轮胎模型类，用于模拟轮胎动态特性，
 * 包括滑移比计算、纵向力计算等功能。
 */

#ifndef TIRE_MODEL_H
#define TIRE_MODEL_H

namespace vehicle_control {

/**
 * @brief 轮胎模型类
 * 
 * 管理轮胎动态特性，模拟轮胎与地面的接触力学
 */
class TireModel {
public:
    /**
     * @brief 构造函数
     * 
     * @param mu 静摩擦系数
     * @param radius 轮胎半径(m)
     * @param width 轮胎宽度(m)
     * @param stiffness 轮胎刚度(N/m)
     * @param damping 阻尼系数(Ns/m)
     * @param cornering_stiffness 侧偏刚度(N/rad)
     */
    TireModel(float mu = 0.8f, 
              float radius = 0.35f, 
              float width = 0.2f,
              float stiffness = 150000.0f,
              float damping = 5000.0f,
              float cornering_stiffness = 25000.0f);
    
    /**
     * @brief 析构函数
     */
    ~TireModel();
    
    /**
     * @brief 更新轮胎负载
     * 
     * @param load 垂直负载力(N)
     */
    void updateLoad(float load);
    
    /**
     * @brief 更新滑移比
     * 
     * @param slip_ratio 滑移比
     */
    void updateSlipRatio(float slip_ratio);
    
    /**
     * @brief 更新输入扭矩
     * 
     * @param torque 输入扭矩(Nm)
     */
    void updateTorqueInput(float torque);
    
    /**
     * @brief 计算纵向力
     * 
     * @return float 纵向力(N)
     */
    float computeLongitudinalForce();
    
    /**
     * @brief 计算横向力
     * 
     * @param slip_angle 侧偏角(rad)
     * @return float 横向力(N)
     */
    float computeLateralForce(float slip_angle);
    
    /**
     * @brief 计算垂向位移
     * 
     * @return float 垂向位移(m)
     */
    float computeVerticalDisplacement();
    
    /**
     * @brief 检查轮胎是否滑动
     * 
     * @return bool 是否滑动
     */
    bool isSliding() const;
    
    /**
     * @brief 设置摩擦系数
     * 
     * @param mu 摩擦系数
     */
    void setFrictionCoefficient(float mu);
    
    /**
     * @brief 设置滑移阈值
     * 
     * @param threshold 滑移阈值
     */
    void setSlipThreshold(float threshold);
    
    /**
     * @brief 获取最大摩擦力
     * 
     * @return float 最大摩擦力(N)
     */
    float getMaxTractionForce() const;
    
    /**
     * @brief 获取滑动状态
     * 
     * @return bool 是否滑动
     */
    bool getSlideState() const;
    
    /**
     * @brief 获取轮胎半径
     * 
     * @return float 轮胎半径(m)
     */
    float getRadius() const;
    
    /**
     * @brief 设置轮胎半径
     * 
     * @param radius 轮胎半径(m)
     */
    void setRadius(float radius);
    
    /**
     * @brief 获取轮胎宽度
     * 
     * @return float 轮胎宽度(m)
     */
    float getWidth() const;
    
    /**
     * @brief 设置轮胎宽度
     * 
     * @param width 轮胎宽度(m)
     */
    void setWidth(float width);
    
    /**
     * @brief 获取轮胎刚度
     * 
     * @return float 轮胎刚度(N/m)
     */
    float getStiffness() const;
    
    /**
     * @brief 设置轮胎刚度
     * 
     * @param stiffness 轮胎刚度(N/m)
     */
    void setStiffness(float stiffness);
    
    /**
     * @brief 获取阻尼系数
     * 
     * @return float 阻尼系数(Ns/m)
     */
    float getDamping() const;
    
    /**
     * @brief 设置阻尼系数
     * 
     * @param damping 阻尼系数(Ns/m)
     */
    void setDamping(float damping);
    
    /**
     * @brief 获取侧偏刚度
     * 
     * @return float 侧偏刚度(N/rad)
     */
    float getCorneringStiffness() const;
    
    /**
     * @brief 设置侧偏刚度
     * 
     * @param cornering_stiffness 侧偏刚度(N/rad)
     */
    void setCorneringStiffness(float cornering_stiffness);
    
    /**
     * @brief 计算接触面积
     * 
     * @return float 接触面积(m^2)
     */
    float computeContactPatchArea() const;
    
    /**
     * @brief 获取滑移比
     * 
     * @return float 滑移比
     */
    float getSlipRatio() const;
    
    /**
     * @brief 获取法向力
     * 
     * @return float 法向力(N)
     */
    float getNormalForce() const;
    
    /**
     * @brief 获取输入扭矩
     * 
     * @return float 输入扭矩(Nm)
     */
    float getInputTorque() const;
    
    /**
     * @brief 获取纵向力
     * 
     * @return float 纵向力(N)
     */
    float getLongitudinalForce() const;
    
private:
    /**
     * @brief 滑移曲线模型
     * 
     * @param slip 滑移比
     * @return float 摩擦系数修正因子
     */
    float slipCurve(float slip);
    
    float mu_;                  ///< 静摩擦系数
    float wheel_radius_;        ///< 轮胎半径(m)
    float wheel_width_;         ///< 轮胎宽度(m)
    float wheel_stiffness_;     ///< 轮胎刚度(N/m)
    float wheel_damping_;       ///< 阻尼系数(Ns/m)
    float cornering_stiffness_; ///< 侧偏刚度(N/rad)
    float normal_force_;        ///< 垂直负载力(N)
    float slip_ratio_;          ///< 滑移比
    float input_torque_;        ///< 输入扭矩(Nm)
    float longitudinal_force_;  ///< 计算的纵向力(N)
    float lateral_force_;       ///< 计算的横向力(N)
    float slip_threshold_;      ///< 滑移阈值
    bool is_sliding_;           ///< 是否处于滑动状态
    float vertical_displacement_; ///< 垂向位移(m)
};

} // namespace vehicle_control

#endif // TIRE_MODEL_H 