#include "fuzzy_PID_controller.h"
namespace fuzzy_pid
{
    FuzzyPID::FuzzyPID()
    {
        // Init
        kp_init = Eigen::Vector3d::Constant(1.1);
        ki_init = Eigen::Vector3d::Constant(0.01);
        kd_init = Eigen::Vector3d::Constant(0.15);
        kp_max = Eigen::Vector3d::Constant(1.5);
        ki_max = Eigen::Vector3d::Constant(0.15);
        kd_max = Eigen::Vector3d::Constant(0.3);
        delta_kp_max = Eigen::Vector3d::Constant(1e-4);
        delta_ki_max = Eigen::Vector3d::Constant(1e-5);
        delta_kd_max = Eigen::Vector3d::Constant(1e-5);
        learn_rate = 1e-6;
        // pre_setpoint = std::numeric_limits<double>::infinity();
        pre_setpoint = Eigen::Vector3d::Constant(0);
    }
    Eigen::Vector3d FuzzyPID::compute(Eigen::Vector3d setpoint, Eigen::Vector3d pv, Eigen::Vector3d current_vel, double dt)
    {
        // 设定新目标点后参数重新初始化
        if ((setpoint - pre_setpoint).norm() > 0.2)
        {
            ki = ki_init;
            kp = kp_init;
            kd = kd_init;
            integral.setZero();
            pre_setpoint = setpoint;
            e_norm = Eigen::Vector3d::Constant(1e-8);
            de_norm = Eigen::Vector3d::Constant(1e-8);
        }
        error = setpoint - pv;
        error_dot = -current_vel;
        delta_kd.setZero();
        delta_ki.setZero();
        delta_kp.setZero();
        for (int i = 0; i < 3; i++)
        {
            // 分离积分
            double integral_OrNot = true;
            if (fabs(error(i)) > 0.2)
            {
                integral(i) = 0;
                integral_OrNot = false;
            }
            else
                integral(i) += error(i) * dt;
            // 更新最大误差和最大误差变量
            if (fabs(error(i)) > e_norm(i))
                e_norm(i) = 1.2 * fabs(error(i));
            if (fabs(error_dot(i)) > de_norm(i))
                de_norm(i) = 1.2 * fabs(error_dot(i));

            membership.clear();
            index.clear();
            // 模糊化输入
            Get_membership(-error(i) / e_norm(i), e_range);
            Get_membership(-error_dot(i) / de_norm(i), de_range);
            // 得到输入对应的隶属度函数
            // 下结合规则得到输出解模糊化
            defuzz_method(i);
            kp(i) += delta_kp(i);
            if (integral_OrNot)
                ki(i) += delta_ki(i);
            else
                ki(i) = ki_init(i);
            // kd(i) += delta_kd(i);
            kp(i) = std::clamp(kp(i), 0.8, kp_max(i));
            ki(i) = std::clamp(ki(i), 0.0, ki_max(i));
            //  kd(i) = std::clamp(kd(i), 0.0, kd_max(i));

            // 加入 alpha 实现低通滤波
            if ((setpoint - pre_setpoint).norm() > 0.2)
            {
                KpOutput(i) = kp(i) * error(i);
                KiOutput(i) = ki(i) * integral(i);
                KdOutput(i) = kd(i) * error_dot(i);
            }
            else
            {
                KpOutput(i) = (1 - alpha) * KpOutput(i) + alpha * kp(i) * error(i);
                KiOutput(i) = (1 - alpha) * KiOutput(i) + alpha * ki(i) * integral(i);
                KdOutput(i) = (1 - alpha) * KdOutput(i) + alpha * kd(i) * error_dot(i);
            }
            // 积分器饱和处理
            if (fabs(KiOutput(i)) > 0.2)
                KiOutput(i) = 0.2 * KiOutput(i) / fabs(KiOutput(i));
        }

        // std::cout<<"output_z:kp"<<kp<<std::endl;
        // std::cout<<"output_z:error"<<error(2)<<std::endl;
        // std::cout<<"output_z:kp"<<KpOutput(2)<<std::endl;
        // std::cout<<"output_z:ki"<<KiOutput(2)<<std::endl;
        // std::cout<<"output_z:kd"<<KdOutput(2)<<std::endl;
        return KpOutput + KiOutput + KdOutput;
    }
    void FuzzyPID::Get_membership(double input, const std::array<float, N> &range)
    {
        // 如果输入值在模糊论域之内
        if (input >= range[0] && input < range[N - 1])
        {
            for (int i = 0; i < N - 1; i++)
            {
                if (input >= range[i] && input < range[i + 1])
                {
                    // membership[0-1] 表示error的模糊集，index[0]表示error的模糊集的编号
                    // index[1]表示error_dot的模糊集的编号，membership[2-3]表示error_dot的模糊集
                    // 顺序取决于先计算error还是error_dot
                    membership.push_back(1 - trimf(input, range[i], range[i + 1]));
                    membership.push_back(trimf(input, range[i], range[i + 1]));
                    index.push_back(i);
                    break;
                }
            }
        }
        // 如果输入值在模糊论域之外
        // 说明输入值过大或过小，此时完全属于某一个模糊集
        else if (input < range[0])
        {
            membership.push_back(1);
            membership.push_back(0);
            index.push_back(0);
        }
        else
        { // if(input >= range[N-1])
            membership.push_back(0);
            membership.push_back(1);
            index.push_back(N - 1);
        }
        // std::cout<<"index: "<<index[0]<<", "<<index[1]<<std::endl;
        // std::cout<<"membership: "<<membership[0]<<", "<<membership[1]<<", "<<membership[2]<<", "<<membership[3]<<std::endl;
    }
    // 将error、error_dot映射到模糊集
    double FuzzyPID::trimf(double input, double min, double max)
    {
        return (input - min) / (max - min);
    }

    void FuzzyPID::defuzz_method(int coordinate_index)
    {
        for (int k = 0; k < 2; k++)
        { // 第一层循环是为了遍历Kp,Ki,Kd三个模糊集
            // 后两层循环是为了遍历两个输入error和error_dot的模糊集
            for (int i = 0; i < M; i++)
            { // error
                for (int j = 0; j < M; j++)
                { // error_dot
                    if (k == 0)
                    {
                        delta_kp(coordinate_index) += membership[i] * membership[2 + j] * kp_rule_matrix[index[0] + i][index[1] + j] * learn_rate;
                        // std::cout<<kp_rule_matrix[index[0] + i][index[1] + j]<<std::endl;
                    }
                    if (k == 1)
                    {
                        delta_ki(coordinate_index) += membership[i] * membership[2 + j] * ki_rule_matrix[index[0] + i][index[1] + j] * learn_rate;
                    }
                    if (k == 2)
                    {
                        delta_kd(coordinate_index) += membership[i] * membership[2 + j] * kd_rule_matrix[index[0] + i][index[1] + j] * learn_rate;
                    }
                }
            }
        }
        delta_kp(coordinate_index) = std::clamp(delta_kp(coordinate_index), -delta_kp_max(coordinate_index), delta_kp_max(coordinate_index));
        delta_ki(coordinate_index) = std::clamp(delta_ki(coordinate_index), -delta_ki_max(coordinate_index), delta_ki_max(coordinate_index));
        delta_kd(coordinate_index) = std::clamp(delta_kd(coordinate_index), -delta_kd_max(coordinate_index), delta_kd_max(coordinate_index));
        // std::cout<<"delta_kp:"<<delta_kp<<std::endl;
        // std::cout<<"delta_kd:"<<delta_kd<<std::endl;
        // std::cout<<"delta_ki:"<<delta_ki<<std::endl;
    }
} // namespace fuzzy_pid
