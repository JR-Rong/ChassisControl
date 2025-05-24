/**
 * @file simple_api_example.cpp
 * @brief 简单接口使用示例程序
 * 
 * 本示例展示了如何使用底盘控制系统的高层API接口。
 * 只需包含chassis_control.h一个头文件即可使用所有功能。
 */

#include "chassis_control.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

// 全局变量，用于处理程序终止信号
volatile sig_atomic_t gRunning = 1;

// 信号处理函数
void signalHandler(int signal) {
    std::cout << "接收到终止信号 " << signal << std::endl;
    gRunning = 0;
}

int main(int argc, char** argv) {
    
    std::cout << "程序正常退出" << std::endl;
    return 0;
    
} 