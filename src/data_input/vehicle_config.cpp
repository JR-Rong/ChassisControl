/**
 * @file vehicle_config.cpp
 * @brief 整车固定参数配置读取类的实现
 */

#include "data_input/vehicle_config.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace chassis_control {
namespace data_input {

VehicleConfig::VehicleConfig() {
    // 初始化车轮位置
    initializeDefaultWheelPositions();
}

VehicleConfig::~VehicleConfig() {
    // 析构函数
}

bool VehicleConfig::loadFromFile(const std::string& yamlPath) {
    try {
        // 检查配置文件是否存在
        std::ifstream file(yamlPath);
        if (!file.good()) {
            std::cerr << "配置文件不存在: " << yamlPath << std::endl;
            return false;
        }
        
        // 加载YAML配置
        YAML::Node config = YAML::LoadFile(yamlPath);
        
        // 读取基本车辆参数
        if (config["vehicle"]) {
            const auto& vehicle = config["vehicle"];
            
            // 读取基本参数
            if (vehicle["mass"]) mass = vehicle["mass"].as<float>();
            if (vehicle["wheelRadius"]) wheelRadius = vehicle["wheelRadius"].as<float>();
            if (vehicle["wheelBase"]) wheelBase = vehicle["wheelBase"].as<float>();
            if (vehicle["trackWidth"]) trackWidth = vehicle["trackWidth"].as<float>();
            if (vehicle["wheelCount"]) wheelCount = vehicle["wheelCount"].as<int>();
            
            // 读取质心位置
            if (vehicle["cogPosition"] && vehicle["cogPosition"].IsSequence() && vehicle["cogPosition"].size() >= 2) {
                cogPosition.first = vehicle["cogPosition"][0].as<float>();
                cogPosition.second = vehicle["cogPosition"][1].as<float>();
            }
            
            // 读取性能参数
            if (vehicle["maxVelocity"]) maxVelocity = vehicle["maxVelocity"].as<float>();
            if (vehicle["maxAcceleration"]) maxAcceleration = vehicle["maxAcceleration"].as<float>();
            if (vehicle["maxDeceleration"]) maxDeceleration = vehicle["maxDeceleration"].as<float>();
            if (vehicle["maxSteeringAngle"]) maxSteeringAngle = vehicle["maxSteeringAngle"].as<float>();
            if (vehicle["maxTorque"]) maxTorque = vehicle["maxTorque"].as<float>();
            if (vehicle["gearRatio"]) gearRatio = vehicle["gearRatio"].as<float>();
            if (vehicle["wheelInfor"]) wheelInfor = vehicle["wheelInfor"].as<float>();
        } else {
            std::cerr << "配置文件中缺少vehicle节点" << std::endl;
            return false;
        }
        
        // 读取车轮位置
        if (config["wheels"] && config["wheels"].IsSequence()) {
            wheelPositions.clear();
            for (const auto& wheel : config["wheels"]) {
                if (wheel.IsSequence() && wheel.size() >= 2) {
                    wheelPositions.push_back({wheel[0].as<float>(), wheel[1].as<float>()});
                }
            }
            
            // 检查车轮位置数量是否与车轮数量一致
            if (static_cast<int>(wheelPositions.size()) != wheelCount) {
                std::cerr << "配置中的车轮位置数量与车轮数量不一致" << std::endl;
                initializeDefaultWheelPositions();  // 如果不一致，则使用默认配置
            }
        }
        
        // 验证参数有效性
        if (!validateParameters()) {
            std::cerr << "车辆参数验证失败" << std::endl;
            return false;
        }
        
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML解析错误: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "初始化错误: " << e.what() << std::endl;
        return false;
    }
}

float VehicleConfig::getMass() const {
    return mass;
}

float VehicleConfig::getWheelRadius() const {
    return wheelRadius;
}

float VehicleConfig::getWheelBase() const {
    return wheelBase;
}

float VehicleConfig::getTrackWidth() const {
    return trackWidth;
}

std::pair<float, float> VehicleConfig::getCoGPosition() const {
    return cogPosition;
}

const std::vector<std::pair<float, float>>& VehicleConfig::getWheelPositions() const {
    return wheelPositions;
}

float VehicleConfig::getWheelInfor() const {
    return wheelInfor;
}

float VehicleConfig::getMaxSteeringAngle() const {
    return maxSteeringAngle;
}

float VehicleConfig::getMaxTorque() const {
    return maxTorque;
}

float VehicleConfig::getMaxVelocity() const {
    return maxVelocity;
}

float VehicleConfig::getMaxAcceleration() const {
    return maxAcceleration;
}

float VehicleConfig::getMaxDeceleration() const {
    return maxDeceleration;
}

float VehicleConfig::getGearRatio() const {
    return gearRatio;
}

void VehicleConfig::printConfig() const {
    std::cout << "======= 车辆配置信息 =======" << std::endl;
    std::cout << "车辆质量: " << mass << " kg" << std::endl;
    std::cout << "车轮半径: " << wheelRadius << " m" << std::endl;
    std::cout << "轴距: " << wheelBase << " m" << std::endl;
    std::cout << "轮距: " << trackWidth << " m" << std::endl;
    std::cout << "车轮数量: " << wheelCount << std::endl;
    std::cout << "质心位置: (" << cogPosition.first << ", " << cogPosition.second << ") m" << std::endl;
    std::cout << "最大速度: " << maxVelocity << " m/s" << std::endl;
    std::cout << "最大加速度: " << maxAcceleration << " m/s^2" << std::endl;
    std::cout << "最大减速度: " << maxDeceleration << " m/s^2" << std::endl;
    std::cout << "最大转向角: " << maxSteeringAngle << " rad" << std::endl;
    std::cout << "最大扭矩: " << maxTorque << " Nm" << std::endl;
    std::cout << "齿轮比: " << gearRatio << std::endl;
    std::cout << "车轮参数: " << wheelInfor << std::endl;
    
    std::cout << "------- 车轮位置信息 -------" << std::endl;
    for (size_t i = 0; i < wheelPositions.size(); ++i) {
        std::cout << "车轮 " << i << ": (" << wheelPositions[i].first << ", " << wheelPositions[i].second << ") m" << std::endl;
    }
    std::cout << "============================" << std::endl;
}

bool VehicleConfig::validateParameters() const {
    // 验证基本参数的有效性
    if (mass <= 0.0f) {
        std::cerr << "车辆质量必须大于0" << std::endl;
        return false;
    }
    
    if (wheelRadius <= 0.0f) {
        std::cerr << "车轮半径必须大于0" << std::endl;
        return false;
    }
    
    if (wheelBase <= 0.0f) {
        std::cerr << "轴距必须大于0" << std::endl;
        return false;
    }
    
    if (trackWidth <= 0.0f) {
        std::cerr << "轮距必须大于0" << std::endl;
        return false;
    }
    
    if (wheelCount <= 0) {
        std::cerr << "车轮数量必须大于0" << std::endl;
        return false;
    }
    
    if (maxTorque <= 0.0f) {
        std::cerr << "最大扭矩必须大于0" << std::endl;
        return false;
    }
    
    if (gearRatio <= 0.0f) {
        std::cerr << "齿轮比必须大于0" << std::endl;
        return false;
    }
    
    return true;
}

void VehicleConfig::initializeDefaultWheelPositions() {
    wheelPositions.clear();
    
    // 默认使用四轮布局
    if (wheelCount == 4) {
        // 前轮
        wheelPositions.push_back({wheelBase/2, trackWidth/2});   // 右前轮
        wheelPositions.push_back({wheelBase/2, -trackWidth/2});  // 左前轮
        
        // 后轮
        wheelPositions.push_back({-wheelBase/2, trackWidth/2});  // 右后轮
        wheelPositions.push_back({-wheelBase/2, -trackWidth/2}); // 左后轮
    } else {
        // 对于其他数量的车轮，创建一个简单的圆形布局
        float radius = std::max(wheelBase, trackWidth) / 2.0f;
        float angleStep = 2.0f * M_PI / wheelCount;
        
        for (int i = 0; i < wheelCount; ++i) {
            float angle = i * angleStep;
            wheelPositions.push_back({radius * std::cos(angle), radius * std::sin(angle)});
        }
    }
}

} // namespace data_input
} // namespace chassis_control 