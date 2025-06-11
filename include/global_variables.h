#ifndef CHASSIS_CONTROL_GLOBAL_VARIABLES_H
#define CHASSIS_CONTROL_GLOBAL_VARIABLES_H

#include <vector>
#include <string>
#include "VehicleControl.h"
#include "VehicleDataInput.h"
#include "can_bus_interface.h"

// 声明全局变量
extern VehicleParam g_vehicle_param;
extern VehicleState g_vehicle_state;
extern ControlMode g_control_mode;
extern std::vector<WheelControlInformation> g_control_info;

// 获取全局变量的函数实现
WheelCharacteristicsCurve& GetWheelCharacteristics(std::string feture_name);
StaticParam& GetStaticParam();
DynamicParam& GetDynamicParam();
VehicleParam& GetVehicleParam();
VehicleState& GetVehicleState();
ControlMode& GetControlMode();
std::vector<WheelControlInformation>& GetControlInfo();

// 设置全局变量的函数实现
void SetDynamicParam(const DynamicParam& value);
void SetVehicleParam(const VehicleParam& value); 
void SetVehicleState(const VehicleState& value); 
void SetControlMode(const ControlMode& value);
void SetControlInfo(const std::vector<WheelControlInformation>& value);  
// void SetCanInfo(const CanBusInformation& value); 

// 初始化全局变量
void InitializeGlobalVariables(const VehicleParam& vehicle,const std::string& can_channel);

// 更新与整车动态参数相关的全局变量
void UpdateGlobalVariables(const DynamicParam& dynamic_param);

// 更新与整车状态相关的全局变量
void UpdateGlobalVariables(const VehicleState& vehicle_state);

#endif // CHASSIS_CONTROL_GLOBAL_VARIABLES_H 