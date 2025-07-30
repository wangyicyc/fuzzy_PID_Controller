
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <fstream> // 包含此头文件以使用文件流
#include <string>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <chrono>
#include "PID_Controller.h"
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include "fuzzy_PID_controller.h"
#include "general.h"
#include "kalman_filter.h"
#include <array>

std::array<KalmanFilter, 3> KFilter;

auto start = std::chrono::high_resolution_clock::now();
auto end = std::chrono::high_resolution_clock::now();
Eigen::Vector3d current_velocity;
mavros_msgs::State current_state;
Eigen::Vector3d setpoint(0.0, 0.0, 0.0);    // 期望位置
Eigen::Vector3d current_pos(0.0, 0.0, 0.0); // 期望位置

fuzzy_pid::FuzzyPID fuzzy_PID_Controller;

enum FLY_MOD
{
    READY,
    GET_POINT,
    TAKEOFF,
    AUTO_LAND,
    MANUAL_LAND,

    END
} state;

struct Point
{
    double x = 0;
    double y = 0;
    double z = 0;
} current_position, home, waypoint[10];

Eigen::Quaterniond q_fcu;

//  圆或H图标
double current_yaw;
double current_roll;
double current_pitch;
double error = 0.30;
double fly_height = 0.4;
double home_yaw;
double omega = M_PI / 10.0; // 角速度（弧度/秒）
// 自定义函数
double error_vel = 0.25;
double tar_height = 0.5;
ros::Time last_request;
double limit_velocity(double expect_vel, int mode)
{
    double limit_vel;
    // mode 决定速度限制为多少
    if (mode == 0)
    {
        if (fabs(expect_vel) > 0.4)
            limit_vel = 0.4 * expect_vel / fabs(expect_vel);
        else
            limit_vel = expect_vel;
    }
    else if (mode == 1)
    {
        if (fabs(expect_vel) > 0.8)
            limit_vel = 0.8 * expect_vel / fabs(expect_vel);
        else
            limit_vel = expect_vel;
    }
    else if (mode == 2)
    {
        if (fabs(expect_vel) > 1.2)
            limit_vel = 1.2 * expect_vel / fabs(expect_vel);
        else
            limit_vel = expect_vel;
    }
    return limit_vel;
}

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    double quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
} // 四元数转换为欧拉角

//  飞控状态
void state_cb(const mavros_msgs::State::ConstPtr &msg) // 回调函数
{
    current_state = *msg;
}
// 获取姿态
double get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    double currTimeSec = time_now.sec - begin.sec;
    double currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

std::array<std::vector<double>, 3> FilterForLoc;
constexpr int windowSize = 5;
void pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    FilterForLoc[0].push_back(msg->pose.pose.position.x);
    FilterForLoc[1].push_back(msg->pose.pose.position.y);
    FilterForLoc[2].push_back(msg->pose.pose.position.z);
    if (FilterForLoc[0].size() == windowSize)
    {
        std::vector<double> data(5);
        // 依次处理XYZ
        for (int i = 0; i < 3; i++)
        {
            data = FilterForLoc[i];
            std::sort(data.begin(), data.end());
            // 中值加均值滤波
            current_pos(i) = (data[(int)(windowSize - 1) / 2] + data[(int)(windowSize - 1) / 2 - 1] + data[(int)(windowSize - 1) / 2 + 1]) / 3;
        }
        FilterForLoc[0].erase(FilterForLoc[0].begin());
        FilterForLoc[1].erase(FilterForLoc[1].begin());
        FilterForLoc[2].erase(FilterForLoc[2].begin());
    }
    current_position.x = current_pos(0);
    current_position.y = current_pos(1);
    current_position.z = current_pos(2);
    q_fcu = Eigen::Quaterniond(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
    // rpy
    current_roll = euler_fcu[0];
    current_pitch = euler_fcu[1];
    current_yaw = euler_fcu[2];
}
std::array<std::vector<double>, 3> FilterForVel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    FilterForVel[0].push_back(msg->twist.linear.x);
    FilterForVel[1].push_back(msg->twist.linear.y);
    FilterForVel[2].push_back(msg->twist.linear.z);
    if (FilterForVel[0].size() == windowSize)
    {
        std::vector<double> data(5);
        // 依次处理XYZ
        for (int i = 0; i < 3; i++)
        {
            data = FilterForVel[i];
            std::sort(data.begin(), data.end());
            // 中值加均值滤波
            current_velocity(i) = (data[(int)(windowSize - 1) / 2] + data[(int)(windowSize - 1) / 2 - 1] + data[(int)(windowSize - 1) / 2 + 1]) / 3;
        }
        FilterForVel[0].erase(FilterForVel[0].begin());
        FilterForVel[1].erase(FilterForVel[1].begin());
        FilterForVel[2].erase(FilterForVel[2].begin());
    }
}
// 计算偏航角（弧度制）

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node"); // 初始化一个名为 "offb_node" 的 ROS 节点。argc 和 argv 是命令行参数，用于传递给 ROS。
    ros::NodeHandle nh;                 // 创建了一个 ROS 节点句柄。
    // Subscribers：
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);                               // 每当接收到新的消息时,将调用 state_cb，获取飞控状态
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, pos_cb);                   // 获取当前位置和姿态作
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, vel_cb); // 获取当前速度

    //  Publishers：
    //  用于发布目标点
    // ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>(
    //     "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Publisher local_vel_pub = nh.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);
    // 解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // 切换飞控模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // ros::init(argc, argv, "offb_node"); // 初始化一个名为 "offb_node" 的 ROS 节点。argc 和 argv 是命令行参数，用于传递给 ROS。
    // ros::NodeHandle nh;                 // 创建了一个 ROS 节点句柄。
    // // Subscribers：
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("iris_0/mavros/state", 1, state_cb);                               // 每当接收到新的消息时,将调用 state_cb，获取飞控状态
    // ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("iris_0/mavros/local_position/odom", 10, pos_cb);                   // 获取当前位置和姿态作
    // ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("iris_0/mavros/local_position/velocity_local", 10, vel_cb); // 获取当前速度
    // ros::Subscriber cir_sub = nh.subscribe<geometry_msgs::Point>("/cir_position", 10, cir_cb);
    // ros::Subscriber match_sub = nh.subscribe<std_msgs::Bool>("/match_result", 10, matchResultCallback);
    // ros::Subscriber end_sub = nh.subscribe<std_msgs::String>("/end", 10, endCallback);
    // //  Publishers：
    // //  用于发布目标点
    // // ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>(
    // //     "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    // ros::Publisher local_vel_pub = nh.advertise<mavros_msgs::PositionTarget>(
    //      "iris_0/mavros/setpoint_raw/local", 10);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
    //     "iris_0/mavros/setpoint_position/local", 10);
    // // 解锁
    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("iris_0/mavros/cmd/arming");
    // // 切换飞控模式
    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("iris_0/mavros/set_mode");
    for (int i = 0; i <= 20; i++)
    {
        nh.param<double>("x" + std::to_string(i), waypoint[i].x, 0.0); // 数值类型（如整数、浮点数等）转换为对应的字符串表示
        nh.param<double>("y" + std::to_string(i), waypoint[i].y, 0.0);
        ROS_INFO("waypoint_x%d: %.2f", i, waypoint[i].x);
        ROS_INFO("waypoint_y%d: %.2f", i, waypoint[i].y);
    }

    Eigen::Vector3d pos_error = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d vel = Eigen::Vector3d(0.0, 0.0, 0.0);
    PID::pos_controller_PID pos_controller_pid;

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0); // 设置循环频率为20Hz
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // 这个 while 循环会持续运行，直到 ROS 系统关闭或无人机成功连接。
    ROS_INFO("Start To Fly!!!");
    ROS_INFO("fly_height:%.2f", fly_height);
    ros::spinOnce();
    rate.sleep();

    // 发布无人机的飞行目标位置（包括当前高度和即将达到的投放高度）。
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = 0;
    goal.pose.position.y = 0;
    goal.pose.position.z = 0;
    goal.pose.orientation.w = 1;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0;
    mavros_msgs::PositionTarget pub_vel;
    pub_vel.velocity.x = 0;
    pub_vel.velocity.y = 0;
    pub_vel.velocity.z = 0;
    pub_vel.yaw = 0;
    pub_vel.type_mask = 0b100111000111;
    pub_vel.coordinate_frame = 1;

    geometry_msgs::PoseStamped pos_setpoint;
    pos_setpoint.pose.position.x = 0;
    pos_setpoint.pose.position.y = 0;
    pos_setpoint.pose.position.z = 0;
    pos_setpoint.pose.orientation.w = 1;
    pos_setpoint.pose.orientation.x = 0;
    pos_setpoint.pose.orientation.y = 0;
    pos_setpoint.pose.orientation.z = 0;

    local_pos_pub.publish(pos_setpoint);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode manual_set_mode;
    manual_set_mode.request.custom_mode = "MANUAL";
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    state = READY;
    // state = TAKEOFF;
    drop_start_time = ros::Time::now();
    ros::Time begin_time = ros::Time::now();
    ros::Time last_request = ros::Time::now();
    double last_time = get_ros_time(begin_time);
    double delta_time = 0;
    double cur_time;

    // send a few setpoints before starting
    // 否则飞控会拒绝接入
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pos_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {

        switch (state)
        {
        case READY:
            // 解锁
            // ROS_INFO("state READY");
            // 距离上次请求超过 0.5 秒
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5)))
            {
                // 切换为OFFBOARD
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            else if (current_state.mode == "OFFBOARD" && current_state.armed)
            {
                ROS_INFO("Vehicle armed");
                // state = TAKEOFF_PLUS;
                state = TAKEOFF;
                ROS_INFO("state TAKEOFF");
                // 获取xyz和yaw
                // home用于记录最初始的状态
                home.x = current_position.x;
                home.y = current_position.y;
                home.z = current_position.z;
                last_request = ros::Time::now();
                break;
            }
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))) // 如果无人机未解锁（!current_state.armed）且从上一次请求解锁到现在的时间超过了0.5秒（ros::Time::now() - last_request > ros::Duration(0.5)），则尝试解锁无人机。
                {
                    if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                        state = TAKEOFF;
                        ROS_INFO("state TAKEOFF");
                        // 获取xyz和yaw
                        // home用于记录最初始的状态
                        home.x = current_position.x;
                        home.y = current_position.y;
                        home.z = current_position.z;
                        home_yaw = current_yaw;
                        // KFilter 设置初始值
                        KFilter[0].init(home.x);
                        KFilter[1].init(home.y);
                        KFilter[2].init(home.z);

                        last_time = get_ros_time(begin_time);
                        end = std::chrono::high_resolution_clock::now();
                        break;
                    }
                    last_request = ros::Time::now();
                }
            }
            break;

        case TAKEOFF:

            pos_error(0) = home.x - current_position.x;
            pos_error(1) = home.y - current_position.y;
            pos_error(2) = home.z + fly_height - current_position.z;
            cur_time = get_ros_time(begin_time);
            delta_time = cur_time - last_time;
            last_time = cur_time;
            // start = std::chrono::high_resolution_clock::now();
            // delta_time为启控到当前时间的间隔
            // delta_time = (start - end).count();
            setpoint(0) = home.x;
            setpoint(1) = home.y;
            setpoint(2) = home.z + fly_height; // 设置期望高度为fly_height
            for (int i = 0; i < 3; i++)
            {
                KFilter[i].update_StateEquation(delta_time);
                KFilter[i].InputUpdate(current_pos(i), current_pos(i));
                KFilter[i].predict();
                current_pos(i) = KFilter[i].getState();
            }
            std::cout << current_pos << std::endl;
            vel = fuzzy_PID_Controller.compute(setpoint, current_pos, current_velocity, delta_time);
            pub_vel.velocity.x = limit_velocity(vel(0), 1);
            pub_vel.velocity.y = limit_velocity(vel(1), 1);
            pub_vel.velocity.z = limit_velocity(vel(2), 1);

            pub_vel.yaw = home_yaw;
            pub_vel.type_mask = 0b100111000111;
            pub_vel.coordinate_frame = 1;
            local_vel_pub.publish(pub_vel);

            if ((fabs(current_position.x - home.x) < error) &&
                (fabs(current_position.y - home.y) < error) &&
                (fabs(current_position.z - home.z - fly_height) < error))
            {
                // goal.pose.position.x = home.x + waypoint[now_goal].x;
                // goal.pose.position.y = home.y + waypoint[now_goal].y;
                // goal.pose.position.z = home.z + fly_height;
            }
            // end = std::chrono::high_resolution_clock::now();
            break;

        case AUTO_LAND:
            // 切auto_land降落

            if (current_state.mode == "OFFBOARD")
            {
                set_mode_client.call(land_set_mode);
                if (land_set_mode.response.mode_sent)
                {
                    ROS_INFO("auto.land enabled");
                    state = END;
                    last_request = ros::Time::now();
                }
            }
            break;

        case MANUAL_LAND:
            if (current_position.z - home.z < 0.05)
            {

                if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5)))
                {
                    if (set_mode_client.call(manual_set_mode) &&
                        manual_set_mode.response.mode_sent)
                    {
                        ROS_INFO("MANUAL enabled");
                        arm_cmd.request.value = false;
                    }
                    last_request = ros::Time::now();
                }
                else if (current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5)))
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle disarmed");
                        state = END;
                        break;
                    }
                }
            }
            break;
        case END:
            // ROS_INFO("MISSION COMPLETE!");
            outFile_x.close();
            outFile_y.close();
            break;
        default:
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}