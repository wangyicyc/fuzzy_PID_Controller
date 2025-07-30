#include "kalman_filter.h"
void KalmanFilter::init(double &x_in){
    z_measurement = VectorXd(1);
    
    x_ = VectorXd(1);
    x_minus = VectorXd(1);
    x_minus(0) = x_in;      // 状态向量
    F_.setIdentity(1,1);
    G_ = MatrixXd(1,1);
    G_ << delta_time;
    H_.setIdentity(1,1);
    u_  = VectorXd(1);
    u_ << 0;
    P_minus = MatrixXd(1,1);
    P_minus << 0.1;
    Q_  = MatrixXd(1,1);
    Q_ << 0.01;
    R_ = MatrixXd(1,1);
    R_ << 0.01;
    
}
// 1.计算卡尔曼增益
void KalmanFilter::Compute_Kalman_Gain(){
    K_ = P_minus * H_.transpose() * (H_ * P_minus * H_.transpose() + R_).inverse();
}
// 2. 依据测量量更新预测状态值
void KalmanFilter::update_estimate(){
    x_ = K_ * (z_measurement - H_ * x_minus) + x_minus;
}
// 3. 更新后验估计误差协方差
void KalmanFilter::update_the_estimate_uncertainty(){
    // 先验估计误差协方差 估计 后验估计误差协方差
    P_ =  (Eigen::MatrixXd::Identity(1, 1) - K_ * H_) *P_minus*(Eigen::MatrixXd::Identity(1, 1) - K_ * H_).transpose();
    P_ += K_ * R_ * K_.transpose();
}
// 4.状态外推函数
void KalmanFilter::Extrapolate_the_State(){
    x_minus = F_ * x_ + G_ * u_;
}
// 5. 更新后验估计误差协方差
void KalmanFilter::Extrapolate_Uncertainty(){
    P_minus = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::predict(){
    Compute_Kalman_Gain();      // 1. 计算卡尔曼增益
    update_estimate();          // 2. 更新估计值
    update_the_estimate_uncertainty();  // 3. 更新估计误差协方差
    Extrapolate_the_State();    // 4. 外推估计状态
    Extrapolate_Uncertainty();  // 5. 更新后验估计误差协方差 
}