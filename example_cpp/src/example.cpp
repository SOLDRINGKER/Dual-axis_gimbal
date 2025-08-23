// src/example.cpp
#include "HaitaiMotorController.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>
#include <csignal>
#include <vector>
#include <string>
#include <map>

// 全局变量用于处理Ctrl+C
volatile bool running = true;

// 信号处理函数
void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n用户中断，正在停止..." << std::endl;
        running = false;
    }
}

// 每个电机的独立PD控制参数
struct MotorPDParams {
    float kp;
    float kd;
};

// 电机内部速度PI控制参数
struct MotorPIParams {
    float kp;
    float ki;
    float filter;
};

// 轨迹参数
struct TrajectoryParams {
    float amplitude;
    float frequency;
    float phase;
};

// PD控制器 - 使用独立的电机PD参数
float pdControl(float targetPosition, float currentPosition, float currentSpeed, 
               const MotorPDParams& pdParams, float maxSpeed) {
    float positionError = targetPosition - currentPosition;
    float speedCommand = pdParams.kp * positionError - pdParams.kd * currentSpeed;
    
    // 速度限幅
    speedCommand = std::max(std::min(speedCommand, maxSpeed), -maxSpeed);
    
    return speedCommand;
}

// 生成正弦目标
float generateSineTarget(float elapsedTime, const TrajectoryParams& params) {
    return params.amplitude * std::sin(2.0f * M_PI * params.frequency * elapsedTime + params.phase);
}

// 绘制ASCII正弦对比图
void drawSineComparisonPlot(const std::vector<float>& timePoints, 
                          const std::vector<float>& targetPoints,
                          const std::vector<float>& actualPoints,
                          const std::string& motorId) {
    const int width = 70;  // 图表宽度
    const int height = 20; // 图表高度
    
    // 找出最大值和最小值
    float maxValue = -1000.0f;
    float minValue = 1000.0f;
    
    for (auto v : targetPoints) {
        maxValue = std::max(maxValue, v);
        minValue = std::min(minValue, v);
    }
    
    for (auto v : actualPoints) {
        maxValue = std::max(maxValue, v);
        minValue = std::min(minValue, v);
    }
    
    // 确保范围足够大
    float range = maxValue - minValue;
    if (range < 1.0f) range = 1.0f;
    
    // 缓冲区，使曲线不贴边
    maxValue += range * 0.1f;
    minValue -= range * 0.1f;
    range = maxValue - minValue;
    
    // 显示电机ID和图表范围
    std::cout << "电机 " << motorId << " 正弦跟随对比图 (T=目标, A=实际)" << std::endl;
    std::cout << "范围: [" << minValue << ", " << maxValue << "]" << std::endl;
    
    // 创建图表
    std::vector<std::string> plot(height, std::string(width, ' '));
    
    // 绘制边框
    for (int i = 0; i < height; i++) {
        plot[i][0] = '|';
        plot[i][width-1] = '|';
    }
    
    for (int j = 0; j < width; j++) {
        plot[0][j] = '-';
        plot[height-1][j] = '-';
    }
    
    // 中线
    int midY = height / 2;
    for (int j = 0; j < width; j++) {
        plot[midY][j] = '-';
    }
    
    // 映射数据点到图表
    for (size_t i = 0; i < timePoints.size() && i < targetPoints.size() && i < actualPoints.size(); i++) {
        // 仅绘制部分点以避免过度拥挤
        if (i % 3 != 0 && i != timePoints.size()-1) continue;
        
        float t = timePoints[i];
        float targetVal = targetPoints[i];
        float actualVal = actualPoints[i];
        
        // 映射到图表坐标
        int x = static_cast<int>((t / timePoints.back()) * (width - 3)) + 1;
        int yTarget = static_cast<int>(((maxValue - targetVal) / range) * (height - 2)) + 1;
        int yActual = static_cast<int>(((maxValue - actualVal) / range) * (height - 2)) + 1;
        
        // 确保在范围内
        x = std::max(1, std::min(x, width - 2));
        yTarget = std::max(1, std::min(yTarget, height - 2));
        yActual = std::max(1, std::min(yActual, height - 2));
        
        // 绘制点
        plot[yTarget][x] = 'T';
        
        // 如果目标和实际位置重合，使用X，否则使用A
        if (yTarget == yActual)
            plot[yActual][x] = 'X';
        else
            plot[yActual][x] = 'A';
    }
    
    // 显示图表
    for (const auto& line : plot) {
        std::cout << line << std::endl;
    }
    
    std::cout << "图例: T=目标位置, A=实际位置, X=位置重合" << std::endl;
    std::cout << std::string(width, '-') << std::endl;
}

// 紧急停止电机
void emergencyStop(HaitaiMotorController& controller, const std::vector<std::string>& motorIds) {
    std::cout << "\n紧急停止电机..." << std::endl;
    
    for (const auto& motorId : motorIds) {
        try {
            HaitaiMotorController::MotorPosition pos;
            controller.speedClosedLoop(motorId, 0, pos);
        } catch (...) {
            // 忽略错误继续停止其他电机
        }
    }
    
    controller.shutdownAllMotors();
    std::cout << "电机已停止" << std::endl;
}

int main(int argc, char* argv[]) {
    // 注册信号处理函数
    std::signal(SIGINT, signalHandler);
    
    // 固定使用ttyCH341USB0串口
    std::string port = "/dev/ttyCH341USB0";
    
    std::cout << "\n========== 海泰双电机正弦跟随演示 ==========\n" << std::endl;
    std::cout << "串口: " << port << std::endl;
    
    try {
        // 电机地址配置
        std::map<std::string, uint8_t> motorAddresses = {
            {"motor1", 0x01},
            {"motor2", 0x02}
        };
        
        // 创建控制器实例
        HaitaiMotorController controller(port, 115200, motorAddresses);
        
        // 每个电机的独立PD控制参数 - 可以在这里修改
        std::map<std::string, MotorPDParams> pdParams = {
            {"motor1", {6.0f, 0.03f}},   // KP=2.5, KD=0.05
            {"motor2", {6.0f, 0.03f}}    // KP=2.5, KD=0.05
        };
        
        // 电机内部PI控制参数 - 可以在这里修改
        MotorPIParams motorPI = {
            0.9f,  // KP
            0.1f,  // KI
            0.9f   // Filter
        };
        
        // 轨迹参数 - 可以在这里修改
        std::map<std::string, TrajectoryParams> trajParams = {
            {"motor1", {100.0f, 1.0f, 0.0f}},       // 振幅100度，1Hz，0相位
            {"motor2", {100.0f, 1.0f, M_PI / 2.0f}} // 振幅100度，1Hz，相位90度
        };
        
        // 控制参数 - 可以在这里修改
        float maxSpeed = 3000.0f;           // 最大速度限制
        float controlFrequency = 100.0f;    // 控制频率100Hz
        float duration = 15.0f;             // 运行时间15秒
        
        // 打印参数配置
        std::cout << "\n参数配置:" << std::endl;
        for (const auto& pair : pdParams) {
            const auto& motorId = pair.first;
            const auto& params = pair.second;
            const auto& traj = trajParams[motorId];
            std::cout << "  " << motorId << " PD控制: Kp=" << params.kp << ", Kd=" << params.kd << std::endl;
            std::cout << "  " << motorId << " 轨迹: 振幅=" << traj.amplitude << "°, 频率=" 
                      << traj.frequency << "Hz, 相位=" << (traj.phase * 180.0f / M_PI) << "°" << std::endl;
        }
        std::cout << "  电机内部PI控制: Kp=" << motorPI.kp << ", Ki=" << motorPI.ki 
                  << ", 滤波=" << motorPI.filter << std::endl;
        std::cout << "  最大速度: " << maxSpeed << " RPM" << std::endl;
        std::cout << "  控制频率: " << controlFrequency << "Hz" << std::endl;
        
        // 打开串口
        std::cout << "\n连接电机控制器..." << std::endl;
        if (!controller.open()) {
            std::cerr << "无法打开串口" << std::endl;
            return 1;
        }
        std::cout << "控制器连接成功" << std::endl;
        
        // 检查电机连接
        std::cout << "\n检查电机连接..." << std::endl;
        std::vector<std::string> connectedMotors;
        
        for (const auto& pair : motorAddresses) {
            try {
                HaitaiMotorController::DeviceInfo info;
                if (controller.getDeviceInfo(pair.first, info) == HaitaiMotorController::ErrorCode::SUCCESS) {
                    std::cout << "发现电机: " << pair.first << std::endl;
                    
                    // 清除故障
                    controller.clearFaults(pair.first);
                    
                    // 配置电机内部PI参数
                    HaitaiMotorController::SystemParameters sysParams;
                    controller.readSystemParams(pair.first, sysParams);
                    
                    // 修改速度PI参数
                    sysParams.speedKp = motorPI.kp;
                    sysParams.speedKi = motorPI.ki;
                    sysParams.speedFilter = motorPI.filter;
                    
                    // 写回修改后的参数
                    controller.writeSystemParamsTemp(pair.first, sysParams);
                    std::cout << "  设置电机内部PI参数: Kp=" << motorPI.kp << ", Ki=" << motorPI.ki << std::endl;
                    
                    // 设置限制
                    HaitaiMotorController::MotorLimits limits = {
                        -180.0f,  // 最小角度
                        180.0f,   // 最大角度
                        maxSpeed  // 最大速度
                    };
                    controller.setMotorLimits(pair.first, limits);
                    
                    // 设置零点
                    controller.setCurrentPositionAsOrigin(pair.first);
                    
                    connectedMotors.push_back(pair.first);
                }
            } catch (const std::exception& e) {
                std::cerr << "连接电机 " << pair.first << " 失败: " << e.what() << std::endl;
            }
        }
        
        if (connectedMotors.empty()) {
            std::cerr << "未找到可用电机，退出程序" << std::endl;
            controller.close();
            return 1;
        }
        
        // 存储数据用于绘图
        struct DataPoints {
            std::vector<float> timePoints;
            std::vector<float> targetPoints;
            std::vector<float> actualPoints;
        };
        
        std::map<std::string, DataPoints> plotData;
        for (const auto& motorId : connectedMotors) {
            plotData[motorId] = DataPoints();
        }
        
        // 控制循环
        std::cout << "\n开始正弦跟随控制（按Ctrl+C停止）...\n" << std::endl;
        
        float loopPeriod = 1.0f / controlFrequency;
        auto startTime = std::chrono::steady_clock::now();
        
        // 主控制循环
        while (running) {
            auto loopStartTime = std::chrono::steady_clock::now();
            float elapsedTime = std::chrono::duration<float>(loopStartTime - startTime).count();
            
            if (elapsedTime > duration) {
                break;
            }
            
            // 控制电机
            for (const auto& motorId : connectedMotors) {
                try {
                    // 生成目标位置 - 使用电机特定的轨迹参数
                    const auto& traj = trajParams[motorId];
                    float targetPosition = generateSineTarget(elapsedTime, traj);
                    
                    // 读取当前状态
                    HaitaiMotorController::MotorPosition position;
                    HaitaiMotorController::MotorStatus status;
                    
                    if (controller.readSystemData(motorId, position, status) == HaitaiMotorController::ErrorCode::SUCCESS) {
                        // PD控制 - 使用电机特定的PD参数
                        float speedCommand = pdControl(
                            targetPosition,
                            position.multiTurnAngle,
                            position.speed,
                            pdParams[motorId],
                            maxSpeed
                        );
                        
                        // 发送速度命令
                        controller.speedClosedLoop(motorId, speedCommand, position);
                        
                        // 记录数据点用于绘图 (每控制周期记录一次)
                        plotData[motorId].timePoints.push_back(elapsedTime);
                        plotData[motorId].targetPoints.push_back(targetPosition);
                        plotData[motorId].actualPoints.push_back(position.multiTurnAngle);
                        
                        // 每秒显示一次状态
                        if (static_cast<int>(elapsedTime * 5) % 5 == 0) {
                            std::cout << "\r时间: " << std::fixed << std::setprecision(1) << elapsedTime 
                                      << "s, " << motorId << ": 目标=" << std::setw(6) << targetPosition 
                                      << "°, 当前=" << std::setw(6) << position.multiTurnAngle 
                                      << "°, 误差=" << std::setw(6) << (targetPosition - position.multiTurnAngle) 
                                      << "°   " << std::flush;
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "控制 " << motorId << " 失败: " << e.what() << std::endl;
                }
            }
            
            // 精确控制循环频率
            auto loopEndTime = std::chrono::steady_clock::now();
            auto loopDuration = std::chrono::duration<float>(loopEndTime - loopStartTime).count();
            float sleepTime = loopPeriod - loopDuration;
            
            if (sleepTime > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleepTime * 1000)));
            }
        }
        
        // 停止所有电机
        emergencyStop(controller, connectedMotors);
        
        // 显示对比图
        std::cout << "\n\n正弦轨迹跟随结果:\n" << std::endl;
        for (const auto& motorId : connectedMotors) {
            const auto& data = plotData[motorId];
            
            // 降采样数据以避免图表过于拥挤
            std::vector<float> sampledTime, sampledTarget, sampledActual;
            size_t step = data.timePoints.size() / 100 + 1;  // 确保不超过100个点
            
            for (size_t i = 0; i < data.timePoints.size(); i += step) {
                sampledTime.push_back(data.timePoints[i]);
                sampledTarget.push_back(data.targetPoints[i]);
                sampledActual.push_back(data.actualPoints[i]);
            }
            
            // 添加最后一个点确保完整性
            if (!data.timePoints.empty() && sampledTime.back() != data.timePoints.back()) {
                sampledTime.push_back(data.timePoints.back());
                sampledTarget.push_back(data.targetPoints.back());
                sampledActual.push_back(data.actualPoints.back());
            }
            
            if (!sampledTime.empty()) {
                drawSineComparisonPlot(sampledTime, sampledTarget, sampledActual, motorId);
                
                // 计算平均误差和最大误差
                float sumError = 0.0f;
                float maxError = 0.0f;
                for (size_t i = 0; i < data.targetPoints.size(); i++) {
                    float error = std::abs(data.targetPoints[i] - data.actualPoints[i]);
                    sumError += error;
                    maxError = std::max(maxError, error);
                }
                float avgError = sumError / data.targetPoints.size();
                
                std::cout << "统计: 平均误差=" << std::fixed << std::setprecision(2) << avgError 
                          << "°, 最大误差=" << maxError << "°\n" << std::endl;
            }
        }
        
        // 关闭控制器
        controller.close();
        
        std::cout << "\n演示完成" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "程序错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
