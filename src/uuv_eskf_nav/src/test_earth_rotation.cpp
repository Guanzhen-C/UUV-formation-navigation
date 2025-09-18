/**
 * @file test_earth_rotation.cpp
 * @brief 地球自转补偿功能测试
 * 
 * 验证ESKF中地球自转补偿的数值正确性，对比KF-GINS的实现
 */

#include "uuv_eskf_nav/eskf_core.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace uuv_eskf_nav;

void testEarthRotationRate() {
    std::cout << "=== 地球自转角速度测试 ===" << std::endl;
    
    // 创建ESKF实例
    NoiseParams noise_params;
    noise_params.gyro_noise_std = 0.01;
    noise_params.accel_noise_std = 0.1;
    noise_params.gyro_bias_std = 1e-5;
    noise_params.accel_bias_std = 1e-4;
    noise_params.dvl_noise_std = 0.02;
    noise_params.depth_noise_std = 0.01;
    
    EskfCore eskf(noise_params);
    
    // 测试不同纬度的地球自转角速度
    std::vector<double> test_latitudes = {0.0, 30.0, 45.0, 60.0, 90.0}; // 度
    
    std::cout << std::fixed << std::setprecision(8);
    std::cout << "纬度(°)   wie_n_x(rad/s)   wie_n_y(rad/s)   wie_n_z(rad/s)" << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;
    
    for (double lat_deg : test_latitudes) {
        double lat_rad = lat_deg * M_PI / 180.0;
        Eigen::Vector3d wie_n = eskf.computeEarthRotationRate(lat_rad);
        
        std::cout << std::setw(6) << lat_deg << "    "
                  << std::setw(13) << wie_n[0] << "   "
                  << std::setw(13) << wie_n[1] << "   " 
                  << std::setw(13) << wie_n[2] << std::endl;
    }
    
    // 验证数值正确性（30°N）
    double lat_30_rad = 30.0 * M_PI / 180.0;
    Eigen::Vector3d wie_n_30 = eskf.computeEarthRotationRate(lat_30_rad);
    
    // 理论值 (参考KF-GINS)
    double expected_x = WGS84_WIE * cos(lat_30_rad);  // 6.3033e-5
    double expected_z = -WGS84_WIE * sin(lat_30_rad); // -3.6461e-5
    
    std::cout << "\n30°N验证:" << std::endl;
    std::cout << "计算值: [" << wie_n_30[0] << ", " << wie_n_30[1] << ", " << wie_n_30[2] << "]" << std::endl;
    std::cout << "理论值: [" << expected_x << ", 0.0, " << expected_z << "]" << std::endl;
    
    bool test_passed = (std::abs(wie_n_30[0] - expected_x) < 1e-10) && 
                       (std::abs(wie_n_30[1]) < 1e-10) &&
                       (std::abs(wie_n_30[2] - expected_z) < 1e-10);
    
    std::cout << "测试结果: " << (test_passed ? "✓ 通过" : "✗ 失败") << std::endl;
}

void testEarthRadii() {
    std::cout << "\n=== 地球椭球半径测试 ===" << std::endl;
    
    NoiseParams noise_params;
    EskfCore eskf(noise_params);
    
    std::vector<double> test_latitudes = {0.0, 30.0, 45.0, 60.0, 90.0}; // 度
    
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "纬度(°)   子午圈半径M(m)   卯酉圈半径N(m)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    for (double lat_deg : test_latitudes) {
        double lat_rad = lat_deg * M_PI / 180.0;
        Eigen::Vector2d radii = eskf.computeEarthRadii(lat_rad);
        
        std::cout << std::setw(6) << lat_deg << "    "
                  << std::setw(15) << radii[0] << "   "
                  << std::setw(15) << radii[1] << std::endl;
    }
    
    // 验证极值
    Eigen::Vector2d radii_equator = eskf.computeEarthRadii(0.0);      // 赤道
    Eigen::Vector2d radii_pole = eskf.computeEarthRadii(M_PI/2.0);    // 北极
    
    std::cout << "\n验证:" << std::endl;
    std::cout << "赤道处: M=" << radii_equator[0] << ", N=" << radii_equator[1] << std::endl;
    std::cout << "北极处: M=" << radii_pole[0] << ", N=" << radii_pole[1] << std::endl;
    
    // 理论值检验
    bool radii_test_passed = (std::abs(radii_equator[1] - WGS84_RA) < 1.0) &&  // N ≈ a at equator
                             (std::abs(radii_pole[0] - radii_pole[1]) < 1.0);     // M ≈ N at pole
    
    std::cout << "测试结果: " << (radii_test_passed ? "✓ 通过" : "✗ 失败") << std::endl;
}

void testCoriolisEffect() {
    std::cout << "\n=== 哥氏力效应测试 ===" << std::endl;
    
    NoiseParams noise_params;
    EskfCore eskf(noise_params);
    
    // 启用地球自转补偿，设置为30°N
    eskf.setEarthRotationParams(true, 30.0 * M_PI / 180.0);
    
    // 模拟一个静止的载体
    NominalState state;
    state.position = Eigen::Vector3d(0, 0, -100);  // 100m深度
    state.velocity = Eigen::Vector3d(1.0, 0, 0);   // 1 m/s 北向速度
    state.orientation = Eigen::Quaterniond::Identity();
    state.gyro_bias.setZero();
    state.accel_bias.setZero();
    
    // 计算30°N的地球自转角速度
    Eigen::Vector3d wie_n = eskf.computeEarthRotationRate(30.0 * M_PI / 180.0);
    
    // 计算哥氏力: -2 * wie_n × vel
    Eigen::Vector3d coriolis_accel = -2.0 * wie_n.cross(state.velocity);
    
    std::cout << std::fixed << std::setprecision(8);
    std::cout << "载体速度: [" << state.velocity.transpose() << "] m/s" << std::endl;
    std::cout << "地球自转角速度: [" << wie_n.transpose() << "] rad/s" << std::endl;
    std::cout << "哥氏加速度: [" << coriolis_accel.transpose() << "] m/s²" << std::endl;
    
    // 对于北向1m/s的运动，在30°N应该产生向东的哥氏力
    // 理论值: a_coriolis_east = 2 * WGS84_WIE * sin(30°) * v_north
    double expected_east_accel = 2.0 * WGS84_WIE * sin(30.0 * M_PI / 180.0) * 1.0;
    
    std::cout << "理论东向哥氏加速度: " << expected_east_accel << " m/s²" << std::endl;
    std::cout << "计算东向哥氏加速度: " << coriolis_accel[1] << " m/s²" << std::endl;
    
    bool coriolis_test_passed = std::abs(coriolis_accel[1] - expected_east_accel) < 1e-10;
    std::cout << "测试结果: " << (coriolis_test_passed ? "✓ 通过" : "✗ 失败") << std::endl;
}

int main() {
    std::cout << "地球自转补偿功能测试" << std::endl;
    std::cout << "===================" << std::endl;
    
    testEarthRotationRate();
    testEarthRadii();
    testCoriolisEffect();
    
    std::cout << "\n测试完成!" << std::endl;
    return 0;
}
