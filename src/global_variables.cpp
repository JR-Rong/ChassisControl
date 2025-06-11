#include "global_variables.h"

// 定义全局变量
VehicleParam g_vehicle_param;
VehicleState g_vehicle_state;
ControlMode g_control_mode;
std::vector<WheelControlInformation> g_control_info;
// CanBusInformation g_can_info;

// 获取全局变量的函数实现
WheelCharacteristicsCurve& GetWheelCharacteristics(const std::string feture_name) { 
    return g_vehicle_param.vechicle_static_param.characteristics_curve[feture_name]; 
}

StaticParam& GetStaticParam() { 
    return g_vehicle_param.vechicle_static_param; 
}

DynamicParam& GetDynamicParam() { 
    return g_vehicle_param.vechicle_dynamic_param; 
}

VehicleParam& GetVehicleParam() { 
    return g_vehicle_param; 
}

VehicleState& GetVehicleState() { 
    return g_vehicle_state; 
}

ControlMode& GetControlMode() { 
    return g_control_mode; 
}

std::vector<WheelControlInformation>& GetControlInfo() { 
    return g_control_info; 
}
// CanBusInformation& GetCanInfo() { return g_can_info; }

void SetVehicleParam(const VehicleParam& value) { 
    g_vehicle_param = value; 
}

void SetDynamicParam(const DynamicParam& value) {
    g_vehicle_param.vechicle_dynamic_param = value; 
}

void SetVehicleState(const VehicleState& value) { 
    g_vehicle_state = value; 
}

void SetControlMode(const ControlMode& value) { 
    g_control_mode = value; 
}

void SetControlInfo(const std::vector<WheelControlInformation>& value) { 
    g_control_info = value; 
}
// void SetCanInfo(const CanBusInformation& value) { g_can_info = value; }

void InitializeGlobalVariables(const VehicleParam& vehicle,const std::string& can_channel) {
    // 初始化车辆参数
    SetVehicleParam(vehicle);
   
    // 初始化车辆状态
    g_vehicle_state = VehicleState();
    
    // 初始化控制模式
    g_control_mode = ControlMode(); // 默认初始化为默认控制模式
} 

// 更新与整车动态参数相关的全局变量
void UpdateGlobalVariables(const DynamicParam& dynamic_param){
    SetDynamicParam(dynamic_param);
}

// 更新与整车状态相关的全局变量
void UpdateGlobalVariables(const VehicleState& vehicle_state){
    SetVehicleState(vehicle_state);
    CalculateWheelControlData(g_vehicle_state, g_vehicle_param);
}