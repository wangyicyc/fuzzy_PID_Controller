#include "fuzzy_PID_controller.h"

namespace fuzzy_pid
{
	FuzzyPID::FuzzyPID()
	{
		learn_rate = 0.001;
		ki = 0.1;
		kp = 0.9;
		kd = 0.1;
		kp_max = 1.5;
		ki_max = 0.5;
		kd_max = 1.0;
		delta_kp_max = 0.01;
		delta_ki_max = 0.001;
		delta_kd_max = 0.001;
		prev_error = 0;
		// pre_setpoint = std::numeric_limits<double>::infinity();
		pre_setpoint = 0;
	}
	// 将error、error_dot映射到模糊集
	double FuzzyPID::trimf(double input, double min, double max)
	{
		return (input - min) / (max - min);
	}

	void FuzzyPID::Get_membership(double input, float *range)
	{
		// 如果输入值在模糊论域之内
		if (input >= range[0] && input < range[N - 1])
		{
			for (int i = 0; i < N - 1; i++)
			{
				if (input >= range[i] && input < range[i + 1])
				{
					// 这样处理，membership[0-1] 表示error的模糊集，index[0]表示error的模糊集的编号
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
		else // if(input >= range[N-1])
		{
			membership.push_back(0);
			membership.push_back(1);
			index.push_back(N - 1);
		}
	}

	void FuzzyPID::defuzz_method()
	{
		for (int k = 0; k < 2; k++) // 第一层循环是为了遍历Kp,Ki,Kd三个模糊集
		{
			// 后两层循环是为了遍历两个输入error和error_dot的模糊集
			for (int i = 0; i < M; i++) // error
			{
				for (int j = 0; j < M; j++) // error_dot
				{
					if (k == 0)
					{
						delta_kp += membership[i] * membership[2 + j] * kp_rule_matrix[index[0] + i][index[1] + j] * learn_rate;
					}
					if (k == 1)
					{
						delta_ki += membership[i] * membership[2 + j] * ki_rule_matrix[index[0] + i][index[1] + j] * learn_rate;
					}
					// if (k == 2)
					// {
					//     delta_kd += membership[i] * membership[2 + j] * kd_rule_matrix[index[0] + i][index[1] + j] * learn_rate;
					// }
				}
			}
		}
		delta_kp = std::clamp(delta_kp, -delta_kp_max, delta_kp_max);
		delta_ki = std::clamp(delta_ki, -delta_ki_max, delta_ki_max);
		// delta_kd = std::clamp(delta_kd,-delta_kd_max,delta_kd_max);
	}
} // namespace fuzzy_pid
