#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include "rules.h"
#include <limits>
#include <algorithm>
#include <Eigen/Eigen>
#include <Eigen/Dense>
namespace fuzzy_pid
{
    class FuzzyPID
    {
    public:
        FuzzyPID();
        Eigen::Vector3d compute(Eigen::Vector3d setpoint, Eigen::Vector3d pv, Eigen::Vector3d current_vel, double dt);
        void get_param()
        {
            std::cout << "kp:" << kp << std::endl;
            std::cout << "ki:" << ki << std::endl;
            std::cout << "kd:" << kd << std::endl;
        }

    private:
        double learn_rate; // 学习率
        // 这里采用重心法解模糊
        void defuzz_method(int coordinate_index);
        // 输入模糊化
        void Get_membership(double input, const std::array<float, N> &arr);
        // 三角隶属度函数
        double trimf(double input, double min, double max);

    public:
        Eigen::Vector3d kp, ki, kd;
        Eigen::Vector3d kp_init, ki_init, kd_init;
        Eigen::Vector3d kp_max, ki_max, kd_max;
        Eigen::Vector3d delta_kp, delta_ki, delta_kd;
        Eigen::Vector3d delta_kp_max, delta_ki_max, delta_kd_max;
        Eigen::Vector3d e_norm{1e-3, 1e-3, 1e-3}, de_norm{1e-3, 1e-3, 1e-3}; // 误差模糊化范围、误差导数模糊化范围
        Eigen::Vector3d error, error_dot;
        Eigen::Vector3d pre_setpoint;
        Eigen::Vector3d integral;
        Eigen::Vector3d KpOutput{0, 0, 0}, KiOutput{0, 0, 0}, KdOutput{0, 0, 0};
        std::vector<double> membership;
        std::vector<int> index;
        // 低通滤波器
        double alpha = 0.8;
    };
} // namespace fuzzy_pid
