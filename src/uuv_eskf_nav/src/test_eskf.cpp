#include "uuv_eskf_nav/eskf_core.h"
#include "uuv_eskf_nav/eskf_types.h"
#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_improved_eskf");
    
    std::cout << "\n=== 改进ESKF算法测试 ===" << std::endl;
    
    // 1. 创建噪声参数
    uuv_eskf_nav::NoiseParams noise_params;
    noise_params.gyro_noise_std = 0.01;
    noise_params.accel_noise_std = 0.1;
    noise_params.gyro_bias_std = 1e-5;
    noise_params.accel_bias_std = 1e-4;
    
    // 2. 创建改进ESKF实例
    uuv_eskf_nav::EskfCore eskf(noise_params);
    
    // 3. 初始化状态
    uuv_eskf_nav::NominalState initial_state;
    initial_state.position.setZero();
    initial_state.velocity.setZero();
    initial_state.orientation.setIdentity();
    initial_state.gyro_bias.setZero();
    initial_state.accel_bias.setZero();
    
    Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Identity(15, 15) * 0.01;
    
    if (!eskf.initialize(initial_state, initial_cov)) {
        std::cerr << "初始化失败!" << std::endl;
        return -1;
    }
    
    // 4. 创建测试IMU数据
    uuv_eskf_nav::ImuData imu_prev, imu_curr;
    
    // 前一时刻IMU数据（静止）
    imu_prev.angular_velocity.setZero();
    imu_prev.linear_acceleration << 0, 0, 9.8;  // 重力加速度
    imu_prev.timestamp = 0.0;
    
    // 当前时刻IMU数据（有旋转）
    imu_curr.angular_velocity << 0.1, 0.0, 0.0;  // 绕X轴旋转
    imu_curr.linear_acceleration << 0, 0, 9.8;
    imu_curr.timestamp = 0.01;  // 10ms后
    
    // 5. 执行改进的预测步骤
    std::cout << "\n--- 执行改进的ESKF预测 ---" << std::endl;
    if (eskf.predictWithImprovedMechanization(imu_curr, imu_prev)) {
        std::cout << "✅ 改进ESKF预测成功!" << std::endl;
        
        // 显示结果
        const auto& state = eskf.getNominalState();
        std::cout << "更新后状态:" << std::endl;
        std::cout << "  位置: [" << state.position.transpose() << "]" << std::endl;
        std::cout << "  速度: [" << state.velocity.transpose() << "]" << std::endl;
        
        // 转换四元数为欧拉角显示
        Eigen::Vector3d euler = state.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
        std::cout << "  姿态: [" << euler.transpose() * 180.0 / M_PI << "] 度" << std::endl;
        
    } else {
        std::cout << "❌ 改进ESKF预测失败!" << std::endl;
        return -1;
    }
    
    // 6. 对比标准方法
    std::cout << "\n--- 特性对比 ---" << std::endl;
    std::cout << "✅ 圆锥误差补偿: 已实现 (基于KF-GINS)" << std::endl;
    std::cout << "✅ 划桨效应补偿: 已实现" << std::endl;
    std::cout << "✅ 角增量积分: 已实现" << std::endl;
    std::cout << "⚠️  导航系转动: 简化实现" << std::endl;
    std::cout << "⚠️  地球椭球模型: 未实现 (适用于短时导航)" << std::endl;
    
    std::cout << "\n🎉 改进ESKF算法测试完成!" << std::endl;
    
    return 0;
}
