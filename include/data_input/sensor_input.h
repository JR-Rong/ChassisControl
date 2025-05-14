/**
 * @file sensor_input.h
 * @brief 传感器数据输入类
 */

#ifndef CHASSIS_CONTROL_SENSOR_INPUT_H
#define CHASSIS_CONTROL_SENSOR_INPUT_H

#include <vector>
#include <string>
#include <array>
#include <memory>

namespace chassis_control {
namespace data_input {

/**
 * @struct CANMessage
 * @brief CAN消息结构体
 */
struct CANMessage {
    uint32_t id;                 ///< CAN ID
    uint8_t data[8];             ///< CAN数据
    uint8_t dlc;                 ///< 数据长度
    uint64_t timestamp;          ///< 时间戳
};

/**
 * @struct VehicleSensorData
 * @brief 车辆传感器数据结构体
 */
struct VehicleSensorData {
    float vehicleSpeed;          ///< 车速(m/s)
    float steeringAngle;         ///< 方向盘转角(rad)
    float acceleration;          ///< 加速度(m/s^2)
    std::vector<float> wheelSpeeds;   ///< 各轮速度(rad/s)
    std::vector<float> vehicleAttitude; ///< 车辆姿态，包含[横摇角,俯仰角,偏航角,横摇角速度,俯仰角速度,偏航角速度]
    uint64_t timestamp;          ///< 数据时间戳
};

/**
 * @class SensorInput
 * @brief 传感器数据输入类，负责从CAN总线获取车辆传感器数据并进行处理
 */
class SensorInput {
public:
    /**
     * @brief 默认构造函数
     */
    SensorInput();

    /**
     * @brief 带配置的构造函数
     * @param can_interface CAN接口名称
     * @param wheel_count 车轮数量
     */
    SensorInput(const std::string& can_interface, int wheel_count = 4);
    
    /**
     * @brief 析构函数
     */
    ~SensorInput();
    
    /**
     * @brief 初始化CAN接口
     * @param can_interface CAN接口名称
     * @return 是否初始化成功
     */
    bool initCANInterface(const std::string& can_interface);
    
    /**
     * @brief 从CAN总线读取数据并更新
     * @return 是否成功更新数据
     */
    bool updateFromCAN();
    
    /**
     * @brief 通过模拟数据更新传感器数据(用于测试)
     * @param speed 车速(m/s)
     * @param steering_angle 方向盘转角(rad)
     * @param accel 加速度(m/s^2)
     * @param wheel_speeds 轮速数组
     * @param attitude 姿态数组
     */
    void updateFromSimulation(float speed, float steering_angle, float accel,
                             const std::vector<float>& wheel_speeds,
                             const std::vector<float>& attitude);
    
    /**
     * @brief 获取车速
     * @return 车速(m/s)
     */
    float getVehicleSpeed() const;
    
    /**
     * @brief 获取方向盘转角
     * @return 方向盘转角(rad)
     */
    float getSteeringAngle() const;
    
    /**
     * @brief 获取加速度
     * @return 加速度(m/s^2)
     */
    float getAcceleration() const;
    
    /**
     * @brief 获取各轮速度
     * @return 轮速数组(rad/s)
     */
    const std::vector<float>& getWheelSpeeds() const;
    
    /**
     * @brief 获取车辆姿态
     * @return 姿态数组[横摇角,俯仰角,偏航角,横摇角速度,俯仰角速度,偏航角速度]
     */
    const std::vector<float>& getVehicleAttitude() const;
    
    /**
     * @brief 获取传感器数据时间戳
     * @return 时间戳
     */
    uint64_t getTimestamp() const;
    
    /**
     * @brief 打印当前传感器数据(用于调试)
     */
    void printSensorData() const;
    
    /**
     * @brief 保存历史数据到文件(用于离线分析)
     * @param filename 文件名
     * @return 是否成功保存
     */
    bool saveDataToFile(const std::string& filename) const;
    
    /**
     * @brief 加载历史数据从文件(用于离线分析)
     * @param filename 文件名
     * @return 是否成功加载
     */
    bool loadDataFromFile(const std::string& filename);

private:
    VehicleSensorData sensorData;        ///< 当前传感器数据
    std::vector<VehicleSensorData> historicalData;  ///< 历史数据
    
    int canSocket;               ///< CAN接口套接字
    std::string canInterface;    ///< CAN接口名称
    bool isCANInitialized;       ///< CAN是否初始化
    
    // CAN解析参数
    struct CANParseConfig {
        uint32_t speedID;            ///< 车速CAN ID
        uint32_t steeringAngleID;    ///< 方向盘转角CAN ID
        uint32_t accelerationID;     ///< 加速度CAN ID
        uint32_t wheelSpeedIDs[6];   ///< 轮速CAN ID
        uint32_t attitudeIDs[2];     ///< 姿态CAN ID
    } canConfig;
    
    /**
     * @brief 从CAN消息解析数据
     * @param msg CAN消息
     * @return 是否解析成功
     */
    bool parseCANMessage(const CANMessage& msg);
    
    /**
     * @brief 解析车速
     * @param msg CAN消息
     */
    void parseVehicleSpeed(const CANMessage& msg);
    
    /**
     * @brief 解析方向盘转角
     * @param msg CAN消息
     */
    void parseSteeringAngle(const CANMessage& msg);
    
    /**
     * @brief 解析加速度
     * @param msg CAN消息
     */
    void parseAcceleration(const CANMessage& msg);
    
    /**
     * @brief 解析轮速
     * @param msg CAN消息
     * @param wheel_index 车轮索引
     */
    void parseWheelSpeed(const CANMessage& msg, int wheel_index);
    
    /**
     * @brief 解析姿态
     * @param msg CAN消息
     * @param attitude_part 姿态部分索引(0:角度, 1:角速度)
     */
    void parseAttitude(const CANMessage& msg, int attitude_part);
    
    /**
     * @brief 获取当前时间戳
     * @return 时间戳(微秒)
     */
    uint64_t getCurrentTimestamp() const;
    
    /**
     * @brief 设置默认CAN解析配置
     */
    void setDefaultCANConfig();
};

} // namespace data_input
} // namespace chassis_control

#endif // CHASSIS_CONTROL_SENSOR_INPUT_H 