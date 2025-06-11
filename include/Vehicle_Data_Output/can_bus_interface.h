/**
 * @file can_bus_interface.h
 * @brief CAN总线通信接口类
 */

#ifndef CHASSIS_CONTROL_CAN_BUS_INTERFACE_H
#define CHASSIS_CONTROL_CAN_BUS_INTERFACE_H

#include <vector>
#include <string>
#include <array>
#include <cstdint>
#include "global_variables.h"

bool SendCanMessage(const std::string& channel_name,int can_id);

#endif // CHASSIS_CONTROL_CAN_BUS_INTERFACE_H 