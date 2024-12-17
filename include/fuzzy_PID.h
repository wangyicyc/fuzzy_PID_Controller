#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include "rules.h"
const double e_norm = 5;  // 误差模糊化范围
const double de_norm = 5; // 误差导数模糊化范围
namespace fuzzy_pid
{
    class FuzzyPID
    {
    public:
        FuzzyPID(double kp, double ki, double kd)
            : kp(kp), ki(ki), kd(kd) {}

        void compute(double setpoint, double pv, double dt)
        {
            error = setpoint - pv;
            integral += error;
            error_dot = (error - prev_error) / dt;
            prev_error = error;
            membership.empty();
            // 模糊化输入
            Get_membership(error / e_norm, e_range);
            Get_membership(error_dot / de_norm, de_range);
            // 得到输入对应的隶属度函数
            // 下结合规则得到输出解模糊化
            delta_kd = 0;
            delta_ki = 0;
            delta_kp = 0;
            defuzz_method();
            kp += delta_kp;
            ki += delta_ki;
            kd += delta_kd;
            kp = kp > kp_max ? kp_max : kp;
            ki = ki > ki_max ? ki_max : ki;
            kd = kd > kd_max ? kd_max : kd;
            kp = kp < kp_min ? kp_min : kp;
        }

    private:
        double kp, ki, kd;
        double kp_min, ki_min, kd_min;
        double kp_max, ki_max, kd_max;
        double delta_kp, delta_ki, delta_kd;
        double delta_kp_max, delta_ki_max, delta_kd_max;
        std::vector<double> membership;
        std::vector<int> index;
        double error;
        double prev_error;
        double error_dot;
        const double learn_rate = 0.1; // 学习率
        double integral;
        // 这里采用重心法解模糊
        void defuzz_method();
        // 输入模糊化
        void Get_membership(double input, float *range);
        // 三角隶属度函数
        double trimf(double input, double min, double max);
    };
} // namespace fuzzy_pid
