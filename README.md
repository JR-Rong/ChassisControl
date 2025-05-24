# ChassisControl 底盘控制系统

## 项目概述

ChassisControl 是一个用于车辆底盘控制的C++库，提供了车辆参数配置、传感器数据处理和状态估计等功能。该库旨在为自动驾驶或智能车辆提供底层控制支持。

## 项目结构

```
ChassisControl/
├── examples/                 # 简单测试目录
│   └── basic_control_example.cpp/    # 检查测试代码 
├── include/                 # 头文件目录
│   └── Vehicle_Control/          # 整车控制模块
|   └── Vehicle_Data_Input/       # 数据输入模块
│   |    └── VehicleDataInput.h    # 整车数据输入头文件
|   └── Vehicle_Data_Output/       # 数据输出模块
├── src/                     # 源文件目录
│   └── Vehicle_Control/          # 整车控制模块
|   └── Vehicle_Data_Input/       # 数据输入模块
│   |    └── VehicleDataInput.h    # 整车数据输入源文件
|   └── Vehicle_Data_Output/       # 数据输出模块
├── chassis_control.h        # 暴露出的接口头文件
├── CMakeLists.txt           # 主构建配置
└── README.md                # 项目说明
```

## 功能模块

### 数据输入模块

数据输入模块包含以下：

1. **VehicleDataInput**: 管理车辆固定参数，如车辆质量、轮距、轴距、车轮半径等。
   - 支持从YAML文件中加载配置
   - 提供参数验证功能
   - 允许获取车辆各种参数

## 依赖项

- C++14或更高版本
- CMake 3.10或更高版本

## 编译和安装

### 1. 安装依赖

#### Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install cmake build-essential libyaml-cpp-dev
```

### 2. 构建项目

```bash
mkdir build && cd build
cmake ..
make
```

### 3. 运行测试

```bash
cd build
make test
```

### 4. 安装

```bash
cd build
sudo make install
```

## 使用示例

```cpp
#include "../chassis_control.h" // 修正头文件路径
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <csignal>
#ifdef _WIN32
#include <windows.h>

#endif

// 全局变量，用于处理程序终止信号
volatile sig_atomic_t gRunning = 1;

int main(int argc, char* argv[]) {
    #ifdef _WIN32
    // 设置控制台输出编码为UTF-8
    SetConsoleOutputCP(CP_UTF8);
    #endif

    std::string config_file = "config.json"; // 配置文件路径
    std::string can_channel = "can0"; // CAN通道
    VehicleParam vehicle_param;
    VehicleState vehicle_state;
    DynamicParam vehicle_dynamic_param;
    
    bool LoadConfigSuccess = LoadConfig(config_file);
    bool InitializeSuccess = Initialize(vehicle_param, can_channel);
    bool UpdateVehicleStateSuccess = UpdateVehicleState(vehicle_state);
    UpdateDynamicParam(vehicle_dynamic_param);
    std::cout << "程序已安全退出" << std::endl;
    return 0;
} 
```

## 许可证

本项目采用MIT许可证。详见[LICENSE](LICENSE)文件。

## 贡献

欢迎提交Pull Request或Issue来改进项目。 