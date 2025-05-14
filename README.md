# ChassisControl 底盘控制系统

## 项目概述

ChassisControl 是一个用于车辆底盘控制的C++库，提供了车辆参数配置、传感器数据处理和状态估计等功能。该库旨在为自动驾驶或智能车辆提供底层控制支持。

## 项目结构

```
ChassisControl/
├── include/                 # 头文件目录
│   └── data_input/          # 数据输入模块头文件
│       ├── vehicle_config.h # 车辆参数配置
│       ├── sensor_input.h   # 传感器输入处理
│       └── state_estimator.h# 状态估计器
├── src/                     # 源文件目录
│   └── data_input/          # 数据输入模块源文件
│       ├── vehicle_config.cpp
│       ├── sensor_input.cpp
│       └── state_estimator.cpp
├── tests/                   # 测试目录
│   ├── test_data/           # 测试数据
│   ├── test_framework.h     # 测试框架
│   ├── test_framework.cpp   # 测试框架实现
│   ├── test_vehicle_config.cpp  # 车辆配置测试
│   └── CMakeLists.txt       # 测试构建配置
├── CMakeLists.txt           # 主构建配置
└── README.md                # 项目说明
```

## 功能模块

### 数据输入模块

数据输入模块包含三个主要类：

1. **VehicleConfig**: 管理车辆固定参数，如车辆质量、轮距、轴距、车轮半径等。
   - 支持从YAML文件中加载配置
   - 提供参数验证功能
   - 允许获取车辆各种参数

2. **SensorInput**: 处理来自各种传感器的输入数据，如陀螺仪、加速度计、轮速传感器等。
   - 提供数据采集和预处理功能
   - 支持传感器数据校准和过滤

3. **StateEstimator**: 基于传感器数据，估计车辆当前状态，如位置、速度、加速度等。
   - 实现多种滤波算法以提高状态估计的精度
   - 提供状态预测功能

## 依赖项

- C++14或更高版本
- CMake 3.10或更高版本
- yaml-cpp库 (用于解析YAML配置文件)

## 编译和安装

### 1. 安装依赖

#### Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install cmake build-essential libyaml-cpp-dev
```

#### Windows (使用vcpkg)
```bash
vcpkg install yaml-cpp
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
#include "data_input/vehicle_config.h"
#include "data_input/sensor_input.h"
#include "data_input/state_estimator.h"
#include <iostream>

int main() {
    // 初始化车辆配置
    chassis_control::data_input::VehicleConfig config;
    if (!config.loadFromFile("vehicle_config.yaml")) {
        std::cerr << "Failed to load vehicle configuration" << std::endl;
        return 1;
    }
    
    // 打印车辆配置
    config.printConfig();
    
    // 创建传感器输入实例
    chassis_control::data_input::SensorInput sensor_input;
    
    // 创建状态估计器
    chassis_control::data_input::StateEstimator estimator;
    
    // 更多功能...
    
    return 0;
}
```

## 许可证

本项目采用MIT许可证。详见[LICENSE](LICENSE)文件。

## 贡献

欢迎提交Pull Request或Issue来改进项目。 