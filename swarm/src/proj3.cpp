/*
 * proj.cpp
 * MS403 Project code3
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <time.h>

#define pi 3.1415926535

using namespace std;
using namespace Eigen;
///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/* 参数定义 */
double r = 5.0;
double c1 = 1.0/6, c2 = 1.0/6, c3 = 1.0/3;
double sigma1 = pi/2, sigma2 = pi/2, sigma3 = pi/12;
double gama = 0.1, Kp = 0.2, k1 = 1, k2 = 0.3;
double z_lock = 2;          // 飞行高度
double k_v = 0.05, k_w = 1;  // 比例系数
double epsilon = 1e-2;
double w = 0.15;
double gain = 1e-0;
//initial position offset
double x_ini[8];
double y_ini[8];
/* 定义变量 */
VectorXd theta(8);
VectorXd phi(8);
VectorXd x(8);
VectorXd y(8);
VectorXd z(8);
VectorXd u_theta(8);
VectorXd u_phi(8);
VectorXd v2(8);
VectorXd z1(8);
VectorXd z2(8);
MatrixXd lap(8, 8);
MatrixXd nei(8, 8);
MatrixXi G(8, 4);

geometry_msgs::PoseStamped pose0;
geometry_msgs::PoseStamped pose1;
geometry_msgs::PoseStamped pose2;
geometry_msgs::PoseStamped pose3;
geometry_msgs::PoseStamped pose4;
geometry_msgs::PoseStamped pose5;
geometry_msgs::PoseStamped pose6;
geometry_msgs::PoseStamped pose7;
nav_msgs::Odometry pose_odom0;
nav_msgs::Odometry pose_odom1;
nav_msgs::Odometry pose_odom2;
nav_msgs::Odometry pose_odom3;
nav_msgs::Odometry pose_odom4;
nav_msgs::Odometry pose_odom5;
nav_msgs::Odometry pose_odom6;
nav_msgs::Odometry pose_odom7;
mavros_msgs::State current_state;

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void update_theta(); // 更新机器人在队形中位置角
void update_state(); // 更新机器人状态，位置+姿态
double get_wij(double theta_i, double theta_j, double phi_i, double phi_j);// 获取连通性权重
void update_lap();                                                         // 更新拉普拉斯矩阵
geometry_msgs::Vector3 quaternion2euler(geometry_msgs::Quaternion quater); // 四元数转欧拉角
geometry_msgs::Quaternion euler2quaternion(geometry_msgs::Vector3 euler);  // 欧拉角转四元数
void update_pub();

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void plane_pos_cb0(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom0 = *msg;
}
void plane_pos_cb1(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom1 = *msg;
}
void plane_pos_cb2(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom2 = *msg;
}
void plane_pos_cb3(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom3 = *msg;
}
void plane_pos_cb4(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom4 = *msg;
}
void plane_pos_cb5(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom5 = *msg;
}
void plane_pos_cb6(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom6 = *msg;
}
void plane_pos_cb7(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom7 = *msg;
}

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    //uav0
    ros::NodeHandle nh0;
    ros::Subscriber state_sub0 = nh0.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub0 = nh0.subscribe<nav_msgs::Odometry>("uav0/mavros/local_position/odom", 1, plane_pos_cb0);
    ros::Publisher local_pos_pub0 = nh0.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client0 = nh0.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client0 = nh0.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");

    //uav1
    ros::NodeHandle nh1;
    ros::Subscriber state_sub1 = nh1.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub1 = nh1.subscribe<nav_msgs::Odometry>("uav1/mavros/local_position/odom", 1, plane_pos_cb1);
    ros::Publisher local_pos_pub1 = nh1.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client1 = nh1.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = nh1.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");

    //uav2
    ros::NodeHandle nh2;
    ros::Subscriber state_sub2 = nh2.subscribe<mavros_msgs::State>("uav2/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub2 = nh2.subscribe<nav_msgs::Odometry>("uav2/mavros/local_position/odom", 1, plane_pos_cb2);
    ros::Publisher local_pos_pub2 = nh2.advertise<geometry_msgs::PoseStamped>("/uav2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client2 = nh2.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client2 = nh2.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");

    //uav3
    ros::NodeHandle nh3;
    ros::Subscriber state_sub3 = nh3.subscribe<mavros_msgs::State>("uav3/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub3 = nh3.subscribe<nav_msgs::Odometry>("uav3/mavros/local_position/odom", 1, plane_pos_cb3);
    ros::Publisher local_pos_pub3 = nh3.advertise<geometry_msgs::PoseStamped>("/uav3/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client3 = nh3.serviceClient<mavros_msgs::CommandBool>("/uav3/mavros/cmd/arming");
    ros::ServiceClient set_mode_client3 = nh3.serviceClient<mavros_msgs::SetMode>("/uav3/mavros/set_mode");

    //uav4
    ros::NodeHandle nh4;
    ros::Subscriber state_sub4 = nh4.subscribe<mavros_msgs::State>("uav4/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub4 = nh4.subscribe<nav_msgs::Odometry>("uav4/mavros/local_position/odom", 1, plane_pos_cb4);
    ros::Publisher local_pos_pub4 = nh4.advertise<geometry_msgs::PoseStamped>("/uav4/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client4 = nh4.serviceClient<mavros_msgs::CommandBool>("/uav4/mavros/cmd/arming");
    ros::ServiceClient set_mode_client4 = nh4.serviceClient<mavros_msgs::SetMode>("/uav4/mavros/set_mode");

    //uav5
    ros::NodeHandle nh5;
    ros::Subscriber state_sub5 = nh5.subscribe<mavros_msgs::State>("uav5/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub5 = nh5.subscribe<nav_msgs::Odometry>("uav5/mavros/local_position/odom", 1, plane_pos_cb5);
    ros::Publisher local_pos_pub5 = nh5.advertise<geometry_msgs::PoseStamped>("/uav5/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client5 = nh5.serviceClient<mavros_msgs::CommandBool>("/uav5/mavros/cmd/arming");
    ros::ServiceClient set_mode_client5 = nh5.serviceClient<mavros_msgs::SetMode>("/uav5/mavros/set_mode");

    //uav6
    ros::NodeHandle nh6;
    ros::Subscriber state_sub6 = nh6.subscribe<mavros_msgs::State>("uav6/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub6 = nh6.subscribe<nav_msgs::Odometry>("uav6/mavros/local_position/odom", 1, plane_pos_cb6);
    ros::Publisher local_pos_pub6 = nh6.advertise<geometry_msgs::PoseStamped>("/uav6/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client6 = nh6.serviceClient<mavros_msgs::CommandBool>("/uav6/mavros/cmd/arming");
    ros::ServiceClient set_mode_client6 = nh6.serviceClient<mavros_msgs::SetMode>("/uav6/mavros/set_mode");

    //uav7
    ros::NodeHandle nh7;
    ros::Subscriber state_sub7 = nh7.subscribe<mavros_msgs::State>("uav7/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub7 = nh7.subscribe<nav_msgs::Odometry>("uav7/mavros/local_position/odom", 1, plane_pos_cb7);
    ros::Publisher local_pos_pub7 = nh7.advertise<geometry_msgs::PoseStamped>("/uav7/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client7 = nh7.serviceClient<mavros_msgs::CommandBool>("/uav7/mavros/cmd/arming");
    ros::ServiceClient set_mode_client7 = nh7.serviceClient<mavros_msgs::SetMode>("/uav7/mavros/set_mode");

    ros::Rate rate(10.0);

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>解 锁 飞 机<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    for (int i = 0; i < 8; ++i){
        x_ini[i] = r*cos(pi/4*i);
        y_ini[i] = r*sin(pi/4*i);
    }

    for (int i = 0; i < 8; ++i){
        G(i, 0) = (i - 2 >= 0) ? i - 2 : i + 6;
        G(i, 1) = (i - 1 >= 0) ? i - 1 : i + 7;
        G(i, 2) = (i + 1 < 8) ? i + 1 : i - 7;
        G(i, 3) = (i + 2 < 8) ? i + 2 : i - 6;
    }

    for (int i = 0; i < 8; ++i){
        for (int j = 0; j < 8; ++j){
            nei(i, j) = 0;
        }
        for (int j = 0; j < 4; ++j){
            nei(i, G(i, j)) = -1;
        }
        nei(i, i) = 4;
    }

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    pose0.pose.position.x = 0;
    pose0.pose.position.y = 0;
    pose0.pose.position.z = z_lock;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = z_lock;
    pose2.pose.position.x = 0;
    pose2.pose.position.y = 0;
    pose2.pose.position.z = z_lock;
    pose3.pose.position.x = 0;
    pose3.pose.position.y = 0;
    pose3.pose.position.z = z_lock;
    pose4.pose.position.x = 0;
    pose4.pose.position.y = 0;
    pose4.pose.position.z = z_lock;
    pose5.pose.position.x = 0;
    pose5.pose.position.y = 0;
    pose5.pose.position.z = z_lock;
    pose6.pose.position.x = 0;
    pose6.pose.position.y = 0;
    pose6.pose.position.z = z_lock;
    pose7.pose.position.x = 0;
    pose7.pose.position.y = 0;
    pose7.pose.position.z = z_lock;

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);
        local_pos_pub3.publish(pose3);
        local_pos_pub4.publish(pose4);
        local_pos_pub5.publish(pose5);
        local_pos_pub6.publish(pose6);
        local_pos_pub7.publish(pose7);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client0.call(offb_set_mode) &&
                set_mode_client1.call(offb_set_mode) &&
                set_mode_client2.call(offb_set_mode) &&
                set_mode_client3.call(offb_set_mode) &&
                set_mode_client4.call(offb_set_mode) &&
                set_mode_client5.call(offb_set_mode) &&
                set_mode_client6.call(offb_set_mode) &&
                set_mode_client7.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled : 8");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client0.call(arm_cmd) &&
                    arming_client1.call(arm_cmd) &&
                    arming_client2.call(arm_cmd) &&
                    arming_client3.call(arm_cmd) &&
                    arming_client4.call(arm_cmd) &&
                    arming_client5.call(arm_cmd) &&
                    arming_client6.call(arm_cmd) &&
                    arming_client7.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed : 8");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);
        local_pos_pub3.publish(pose3);
        local_pos_pub4.publish(pose4);
        local_pos_pub5.publish(pose5);
        local_pos_pub6.publish(pose6);
        local_pos_pub7.publish(pose7);
        if (pose_odom0.pose.pose.position.z > 1.8) {
            ROS_INFO("plane takeoff !");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    while (ros::ok()) {
        update_state();
        update_lap();
        // estimate v2
        double tp1, tp2;
        v2(7) = 0;
        for (int i = 0; i < 7; ++i){
            v2(i) = rand() / double(RAND_MAX);
            v2(7) -= v2(i);
        }
        tp2 = v2.array().square().matrix().sum();
        v2 = ( v2.array()/sqrt(tp2) ).matrix();
        /*
        for (int i = 0; i < 8; ++i){
            z1(i) = 0.0;
            z2(i) = 1.0/8;
        }
        for (int i = 0; i < 10; ++i){
            z1 += gama*(v2 - z1) - Kp*nei*z1;
            z2 += gama*(v2.array().square().matrix() - z2) - Kp*nei*z2;
            v2 -= k1*z1 + k2*lap*v2;
            for (int i = 0; i < 8; ++i){
                v2(i) = v2(i)/sqrt(z2(i)*8);
            }
        }
        */
       for (int i = 0; i < 20; ++i){
            tp1 = v2.sum();
            v2 = ( v2.array() - tp1/8 ).matrix() - k2*lap*v2;
            tp2 = v2.array().square().matrix().sum();
            v2 = ( v2.array()/sqrt(tp2) ).matrix();
        }
        EigenSolver<Eigen::MatrixXd> es(lap);
        MatrixXcd evals = es.eigenvalues();
        VectorXd evalsReal = evals.real();
        double lambda1, lambda2;
        if (evalsReal(0) < evalsReal(1)){
            lambda1 = evalsReal(0);
            lambda2 = evalsReal(1);
        }
        else {
            lambda1 = evalsReal(1);
            lambda2 = evalsReal(0);
        }
        for (int i = 2; i < 8; ++i){
            if (evalsReal(i) < lambda1){
                lambda2 = lambda1;
                lambda1 = evalsReal(i);
            }
            else {
                if (evalsReal(i) < lambda2){
                    lambda2 = evalsReal(i);
                }
            }
        }
        cout << "threshold:" << (lap*v2 - lambda2*v2).norm() << endl;
        // calculate u
        for (int i = 0; i < 8; ++i){
            u_theta(i) = 0;
            u_phi(i) = 0;
            for (int j = 0; j < 4; ++j){
                u_theta(i) += (get_wij(theta(i) + epsilon, theta(G(i, j)), phi(i), phi(G(i, j))) + lap(i, G(i, j)))/epsilon*(v2(i) - v2(G(i, j)))*(v2(i) - v2(G(i, j)))*gain;
                u_phi(i) += (get_wij(theta(i), theta(G(i, j)), phi(i) + epsilon, phi(G(i, j))) + lap(i, G(i, j)))/epsilon*(v2(i) - v2(G(i, j)))*(v2(i) - v2(G(i, j)))*gain;
            }
            //cout << "uav" << i << ' ' << u_theta(i) << ' ' << u_phi(i) << endl;
        }
        // publish control
        update_pub();
        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);
        local_pos_pub3.publish(pose3);
        local_pos_pub4.publish(pose4);
        local_pos_pub5.publish(pose5);
        local_pos_pub6.publish(pose6);
        local_pos_pub7.publish(pose7);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void update_theta(){
    for (int i = 0; i < 8; ++i){
        if (x(i) > 0){
            theta(i) = atan(y(i)/x(i));
        }
        else {
            if (y(i) > 0){
                theta(i) = atan(y(i)/x(i)) + pi;
            }
            else {
                theta(i) = atan(y(i)/x(i)) - pi;
            }
        }
    }
}

void update_state(){
    x(0) = pose_odom0.pose.pose.position.x + x_ini[0];
    y(0) = pose_odom0.pose.pose.position.y + y_ini[0];
    z(0) = pose_odom0.pose.pose.position.z;
    phi(0) = quaternion2euler(pose_odom0.pose.pose.orientation).z;
    x(1) = pose_odom1.pose.pose.position.x + x_ini[1];
    y(1) = pose_odom1.pose.pose.position.y + y_ini[1];
    z(1) = pose_odom1.pose.pose.position.z;
    phi(1) = quaternion2euler(pose_odom1.pose.pose.orientation).z;
    x(2) = pose_odom2.pose.pose.position.x + x_ini[2];
    y(2) = pose_odom2.pose.pose.position.y + y_ini[2];
    z(2) = pose_odom2.pose.pose.position.z;
    phi(2) = quaternion2euler(pose_odom2.pose.pose.orientation).z;
    x(3) = pose_odom3.pose.pose.position.x + x_ini[3];
    y(3) = pose_odom3.pose.pose.position.y + y_ini[3];
    z(3) = pose_odom3.pose.pose.position.z;
    phi(3) = quaternion2euler(pose_odom3.pose.pose.orientation).z;
    x(4) = pose_odom4.pose.pose.position.x + x_ini[4];
    y(4) = pose_odom4.pose.pose.position.y + y_ini[4];
    z(4) = pose_odom4.pose.pose.position.z;
    phi(4) = quaternion2euler(pose_odom4.pose.pose.orientation).z;
    x(5) = pose_odom5.pose.pose.position.x + x_ini[5];
    y(5) = pose_odom5.pose.pose.position.y + y_ini[5];
    z(5) = pose_odom5.pose.pose.position.z;
    phi(5) = quaternion2euler(pose_odom5.pose.pose.orientation).z;
    x(6) = pose_odom6.pose.pose.position.x + x_ini[6];
    y(6) = pose_odom6.pose.pose.position.y + y_ini[6];
    z(6) = pose_odom6.pose.pose.position.z;
    phi(6) = quaternion2euler(pose_odom6.pose.pose.orientation).z;
    x(7) = pose_odom7.pose.pose.position.x + x_ini[7];
    y(7) = pose_odom7.pose.pose.position.y + y_ini[7];
    z(7) = pose_odom7.pose.pose.position.z;
    phi(7) = quaternion2euler(pose_odom7.pose.pose.orientation).z;
    update_theta();
}

geometry_msgs::Vector3 quaternion2euler(geometry_msgs::Quaternion quater){
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (quater.w * quater.x + quater.y * quater.z), 1.0 - 2.0 * (quater.x * quater.x + quater.y * quater.y));
    temp.y = asin(2.0 * (quater.w * quater.y - quater.z * quater.x));
    temp.z = atan2(2.0 * (quater.w * quater.z + quater.x * quater.y), 1.0 - 2.0 * (quater.y * quater.y + quater.z * quater.z));
    return temp;
}

geometry_msgs::Quaternion euler2quaternion(geometry_msgs::Vector3 euler)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(euler.x/2)*cos(euler.y/2)*cos(euler.z/2) + sin(euler.x/2)*sin(euler.y/2)*sin(euler.z/2);
    temp.x = sin(euler.x/2)*cos(euler.y/2)*cos(euler.z/2) - cos(euler.x/2)*sin(euler.y/2)*sin(euler.z/2);
    temp.y = cos(euler.x/2)*sin(euler.y/2)*cos(euler.z/2) + sin(euler.x/2)*cos(euler.y/2)*sin(euler.z/2);
    temp.z = cos(euler.x/2)*cos(euler.y/2)*sin(euler.z/2) - sin(euler.x/2)*sin(euler.y/2)*cos(euler.z/2);
    return temp;
}

void update_pub(){
    geometry_msgs::Vector3 euler;
    euler = quaternion2euler(pose_odom0.pose.pose.orientation);
    euler.z += k_w*u_phi(0);
    pose0.pose.orientation = euler2quaternion(euler);
    euler = quaternion2euler(pose_odom1.pose.pose.orientation);
    euler.z += k_w*u_phi(1);
    pose1.pose.orientation = euler2quaternion(euler);
    euler = quaternion2euler(pose_odom2.pose.pose.orientation);
    euler.z += k_w*u_phi(2);
    pose2.pose.orientation = euler2quaternion(euler);
    euler = quaternion2euler(pose_odom3.pose.pose.orientation);
    euler.z += k_w*u_phi(3);
    pose3.pose.orientation = euler2quaternion(euler);
    euler = quaternion2euler(pose_odom4.pose.pose.orientation);
    euler.z += k_w*u_phi(4);
    pose4.pose.orientation = euler2quaternion(euler);
    euler = quaternion2euler(pose_odom5.pose.pose.orientation);
    euler.z += k_w*u_phi(5);
    pose5.pose.orientation = euler2quaternion(euler);
    euler = quaternion2euler(pose_odom6.pose.pose.orientation);
    euler.z += k_w*u_phi(6);
    pose6.pose.orientation = euler2quaternion(euler);
    euler = quaternion2euler(pose_odom7.pose.pose.orientation);
    euler.z += k_w*u_phi(7);
    pose7.pose.orientation = euler2quaternion(euler);
    pose0.pose.position.x = r*cos(theta(0) + k_v*u_theta(0) + w) - x_ini[0];
    pose1.pose.position.x = r*cos(theta(1) + k_v*u_theta(1) + w) - x_ini[1];
    pose2.pose.position.x = r*cos(theta(2) + k_v*u_theta(2) + w) - x_ini[2];
    pose3.pose.position.x = r*cos(theta(3) + k_v*u_theta(3) + w) - x_ini[3];
    pose4.pose.position.x = r*cos(theta(4) + k_v*u_theta(4) + w) - x_ini[4];
    pose5.pose.position.x = r*cos(theta(5) + k_v*u_theta(5) + w) - x_ini[5];
    pose6.pose.position.x = r*cos(theta(6) + k_v*u_theta(6) + w) - x_ini[6];
    pose7.pose.position.x = r*cos(theta(7) + k_v*u_theta(7) + w) - x_ini[7];
    pose0.pose.position.y = r*sin(theta(0) + k_v*u_theta(0) + w) - y_ini[0];
    pose1.pose.position.y = r*sin(theta(1) + k_v*u_theta(1) + w) - y_ini[1];
    pose2.pose.position.y = r*sin(theta(2) + k_v*u_theta(2) + w) - y_ini[2];
    pose3.pose.position.y = r*sin(theta(3) + k_v*u_theta(3) + w) - y_ini[3];
    pose4.pose.position.y = r*sin(theta(4) + k_v*u_theta(4) + w) - y_ini[4];
    pose5.pose.position.y = r*sin(theta(5) + k_v*u_theta(5) + w) - y_ini[5];
    pose6.pose.position.y = r*sin(theta(6) + k_v*u_theta(6) + w) - y_ini[6];
    pose7.pose.position.y = r*sin(theta(7) + k_v*u_theta(7) + w) - y_ini[7];
    pose0.pose.position.z = z_lock;
    pose1.pose.position.z = z_lock;
    pose2.pose.position.z = z_lock;
    pose3.pose.position.z = z_lock;
    pose4.pose.position.z = z_lock;
    pose5.pose.position.z = z_lock;
    pose6.pose.position.z = z_lock;
    pose7.pose.position.z = z_lock;
}

double get_wij(double theta_i, double theta_j, double phi_i, double phi_j){
    double x_i = r*cos(theta_i);
    double y_i = r*sin(theta_i);
    double x_j = r*cos(theta_j);
    double y_j = r*sin(theta_j);
    double psi_ij, psi_ji;
    if (x_j - x_i > 0){
        psi_ij = atan((y_j - y_i)/(x_j - x_i));
    }
    else {
        if (y_j - x_i > 0){
            psi_ij = atan((y_j - y_i)/(x_j - x_i)) + pi;
        }
        else {
            psi_ij = atan((y_j - y_i)/(x_j - x_i)) - pi;
        }
    }
    if (x_i - x_j > 0){
        psi_ji = atan((y_i - y_j)/(x_i - x_j));
    }
    else {
        if (y_i - x_j > 0){
            psi_ji = atan((y_i - y_j)/(x_i - x_j)) + pi;
        }
        else {
            psi_ji = atan((y_i - y_j)/(x_i - x_j)) - pi;
        }
    }
    return c1*exp(-(psi_ij - phi_i)*(psi_ij - phi_i)/sigma1/sigma1) + c2*exp(-phi_i*phi_i/sigma2/sigma2)\
     + c3*exp(-(fabs(theta_j - theta_i) - pi/4)*(fabs(theta_j - theta_i) - pi/4)/sigma3/sigma3) + \
     c1*exp(-(psi_ji - phi_j)*(psi_ji - phi_j)/sigma1/sigma1) + c2*exp(-phi_j*phi_j/sigma2/sigma2)\
     + c3*exp(-(fabs(theta_j - theta_i) - pi/4)*(fabs(theta_j - theta_i) - pi/4)/sigma3/sigma3);
}

void update_lap(){
    for (int i = 0; i < 8; ++i){
        for (int j = 0; j < 8; ++j){
            lap(i, j) = 0;
        }
    }
    for (int i = 0; i < 8; ++i){
        for (int j = 0; j < 2; ++j){
            lap(i, G(i, j)) = -get_wij(theta(i), theta(G(i, j)), phi(i), phi(G(i, j)));
            lap(G(i, j), i) = lap(i, G(i, j));
        }
    }
    for (int i = 0; i < 8; ++i){
        for (int j = 0; j < 8; ++j){
            if (i == j){
                continue;
            }
            else {
                lap(i, i) -= lap(i, j);
            }
        }
    }
}