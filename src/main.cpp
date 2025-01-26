#include <iostream>
#include <chrono>
#include "fuzzy_PID_controller.h"
#include <fstream>
int main()
{
    // 创建一个输出文件流对象，并指定文件名
    std::ofstream outFile;
    // 打开文件（如果文件不存在会创建新文件）
    outFile.open("output.txt");
    if (!outFile.is_open())
    {
        std::cerr << "Failed to open file." << std::endl;
        return 1;
    }

    fuzzy_pid::FuzzyPID pid;
    double setpoint = 1;
    double pv = 0;
    double dt = 0.001;
    auto start = std::chrono::high_resolution_clock::now();
    pv += pid.compute(setpoint, pv, dt);
    double used_time;
    while (true)
    {
        pv += pid.compute(setpoint, pv, dt);
        if (fabs(pv - setpoint) < 1e-3)
        {
            setpoint += 30;
            auto end = std::chrono::high_resolution_clock::now();
            pid.get_param();
            used_time = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
            // printf("used_time: %.3f",used_time);
            std::cout << std::endl;
            // if(setpoint>3)
            //   break;
        }
        // if(fabs(pv) > 2.5)
        // {
        //     break;
        // }
        // outFile << "pv: " << pv << std::endl;
        // outFile << "de: " << pid.error_dot << std::endl;
        // outFile << "de_norm: " << pid.de_norm << std::endl;
        // // outFile << "kp: " << pid.kp << std::endl;
        // outFile << "delta_kp: " << pid.delta_kp << std::endl;
        // for(int i = 0; i< pid.index.size() ; i++)
        // {
        //     outFile << "index"<<i<<": "<< pid.index[i] << std::endl;
        // }
        // for(int i = 0; i< pid.membership.size() ; i++)
        // {
        //     outFile << "membership"<<i<<": "<< pid.membership[i] << std::endl;
        // }
        // outFile << "ki: " << pid.ki << std::endl;
        std::cout << "pv: " << pv << std::endl;
        pid.get_param();
        // std::cout << "used_time: " << used_time << std::endl;
    }
    pid.get_param();
    outFile.close();
    return 0;
}
