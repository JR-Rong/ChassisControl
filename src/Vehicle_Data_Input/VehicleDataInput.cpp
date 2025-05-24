#include <iostream>
#include "Vehicle_Data_Input/VehicleDataInput.h" // Adjust the path based on the actual location of the header file

/**
 * @brief 从配置文件中加载配置
 * @param config_file 配置文件路径
 * @return bool 是否成功加载
 */
bool LoadConfig(const std::string& config_file){
    std::cout << "LoadConfig" << std::endl;
    return true;
}

/**
 * @brief 初始化
 * @param vehicle 整车参数
 * @param can_channel CAN通道
 * @return bool 是否初始化成功
 */
bool Initialize(const VehicleParam& vehicle, const std::string& can_channel){
    std::cout << "Initialize" << std::endl;
    return true;
}

/**
 * @brief 更新车辆状态
 * @param vehicle_state 当前车辆运行状态
 * @return bool 更新是否成功
 */
bool UpdateVehicleState(const VehicleState& vehicle_state){
    std::cout << "UpdateVehicleState" << std::endl;
    return true;
}
/**
 * @brief 更新整车参数
 * @param vehicle_dyanmic_param 待更新的整车参数
 */
void UpdateDynamicParam(const DynamicParam& vehicle_dyanmic_param){
    std::cout << "UpdateDynamicParam" << std::endl;
    return ;
}