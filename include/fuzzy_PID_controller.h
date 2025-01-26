#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include "rules.h"
#include <limits>
#include <algorithm>
namespace fuzzy_pid
{
    class FuzzyPID
    {
    public:
        FuzzyPID();
        double compute(double setpoint, double pv, double dt)
        {
            // 设定新目标点后参数重新初始化
            if (setpoint != pre_setpoint)
            {
                integral = 0;
                prev_error = 0;
                pre_setpoint = setpoint;
                e_norm = 1e-8;
                de_norm = 1e-8;
            }

            error = setpoint - pv;
            // 如果是新开一个点，则de先不计入
            if (prev_error != 0)
                error_dot = (error - prev_error) / dt;
            else
                error_dot = 0;

            // 分离积分
            if (fabs(error) > 0.3)
                integral = 0;
            else
                integral += error * dt;
            // 更新最大误差和最大误差变量
            if (fabs(error) > e_norm)
            {
                e_norm = 1.2 * fabs(error);
            }
            if (fabs(error_dot) > de_norm)
            {
                de_norm = 1.2 * fabs(error_dot);
            }

            prev_error = error;
            membership.clear();
            index.clear();
            // 模糊化输入
            Get_membership(error / e_norm, e_range);
            Get_membership(error_dot / de_norm, de_range);
            // 得到输入对应的隶属度函数
            // 下结合规则得到输出解模糊化
            delta_kd = 0;
            delta_ki = 0;
            // delta_kp = 0;
            defuzz_method();
            kp += delta_kp;
            ki += delta_ki;
            // kd += delta_kd;
            kp = std::clamp(kp, -kp_max, kp_max);
            ki = std::clamp(ki, -ki_max, ki_max);
            // kd = fabs(kd) > kd_max ? kd * kd_max / fabs(kd) : kd;
            return (kp * error + ki * integral + kd * error_dot) * dt;
        }
        void get_param()
        {

            std::cout << "kp:" << kp << std::endl;
            std::cout << "ki:" << ki << std::endl;
            std::cout << "kd:" << kd << std::endl;
            // std::cout<<"delta_kp:"<<delta_kp<<std::endl;
            // std::cout<<"delta_kd:"<<delta_kd<<std::endl;
            // std::cout<<"delta_ki:"<<delta_ki<<std::endl;
            // std::cout<<"index:"<<index.size()<<std::endl;
            // std::cout<<"membership:"<<membership.size()<<std::endl;
            // for(int i=0;i<index.size();i++)
            // {
            //    std::cout<<"index"<<i<<":"<<index[i]<<std::endl;
            // }
            // for(int i=0;i<membership.size();i++)
            // {
            //    std::cout<<"membership"<<i<<":"<<membership[i]<<std::endl;
            // }
        }

    private:
        double learn_rate; // 学习率
        // 这里采用重心法解模糊
        void defuzz_method();
        // 输入模糊化
        void Get_membership(double input, float *range);
        // 三角隶属度函数
        double trimf(double input, double min, double max);

    public:
        double kp, ki, kd;
        double kp_max, ki_max, kd_max;
        double delta_kp, delta_ki, delta_kd;
        double delta_kp_max, delta_ki_max, delta_kd_max;
        double e_norm = 1e-3, de_norm = 1e-3; // 误差模糊化范围、误差导数模糊化范围
        double error, prev_error, error_dot;
        double pre_setpoint;
        double integral;
        std::vector<double> membership;
        std::vector<int> index;
    };
} // namespace fuzzy_pid
