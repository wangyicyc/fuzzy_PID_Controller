#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
using Eigen::MatrixXd;
using Eigen::VectorXd;
class KalmanFilter
{
public:
    // 构造函数
    KalmanFilter() {}
    // 初始化卡尔曼滤波器
    void init(double &x_in);
    // 1.计算卡尔曼增益
    void Compute_Kalman_Gain();
    // 2. 依据测量量更新预测状态值
    void update_estimate();
    // 3. 更新后验估计误差协方差
    void update_the_estimate_uncertainty();
    // 4.状态外推函数
    void Extrapolate_the_State();
    // 5. 更新后验估计误差协方差
    void Extrapolate_Uncertainty();
    // 联合使用以上函数
    void predict();
    // 获取当前状态
    double getState() const { return x_(0); }
    // 更新输入
    void InputUpdate(double u,double z) { 
        z_measurement(0) = z;
        u_(0) = u;
    }
    void update_StateEquation(double dtime){
        G_(0,0) = dtime;
    }

private:
    VectorXd x_;      // 后验估计状态
    VectorXd x_minus; // 先验状态
    MatrixXd K_;      // 卡尔曼增益
    MatrixXd P_;      // 后验估计误差协方差矩阵
    MatrixXd P_minus; // 先验估计误差协方差矩阵
    MatrixXd F_;      // 状态转移矩阵，表示x_的状态随时间变化的关系
    MatrixXd G_;      // 输入矩阵
    MatrixXd R_n;     // 测量噪声矩阵
    MatrixXd H_;      // 观测矩阵，表示x_和y_之间的关系
    MatrixXd R_;      // 测量噪声矩阵
    MatrixXd Q_;      // 过程噪声矩阵协方差
    VectorXd z_measurement;
    VectorXd u_;
    double delta_time  = 0.1;   // 采样间隔
};
