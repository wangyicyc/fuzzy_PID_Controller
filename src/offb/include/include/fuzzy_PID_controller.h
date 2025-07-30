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
        double compute(double setpoint, double pv, double dt);
    
        void get_param()
        {

            std::cout<<"kp:"<<kp<<std::endl;
            std::cout<<"ki:"<<ki<<std::endl;
            std::cout<<"kd:"<<kd<<std::endl;
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
        void Get_membership(double input, const std::array<float, N>& arr);
        // 三角隶属度函数
        double trimf(double input, double min, double max);
    public:
        double kp, ki, kd;
        double kp_max, ki_max, kd_max;
        double delta_kp, delta_ki, delta_kd;
        double delta_kp_max, delta_ki_max, delta_kd_max;
        double e_norm = 1e-3,de_norm = 1e-3;  // 误差模糊化范围、误差导数模糊化范围
        double error,prev_error,error_dot;
        double pre_setpoint;
        double integral;
        std::vector<double> membership;
        std::vector<int> index;
    };
} // namespace fuzzy_pid
