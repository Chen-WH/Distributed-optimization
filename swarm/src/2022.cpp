/*
 * 2022 Spring Festival
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>

#define pi 3.1415926535

using namespace std;
using namespace Eigen;
///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/* 第一阶段参数 */
double epsilon = 0.05;
int point1 = 8;
double yaw1[4][8] = {
    0, 0, 0, 0, 0, 0, 0, pi,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};
double x_1[4][8] = {
      0,   0,  0,   0,   0, -2.5,   0,   0,
    -15, -15, -6, -15, -15,  -15, -15, -15,
     15,   6,  6,  15,  15,  2.5,  15,  15,
     15,  15, 15,  15,  15,   15,  15,  15
};
double y_1[4][8] = {
      0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,
    -10, -10, -10, -10, -10, -10, -10, -10
};
double z_1[4][8] = {
    12, 12, 12, 12, 20, 20, 20, 20,
    12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 20, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12
};
/* 第二阶段参数 */
int point2 = 2;
double yaw2[4][2] = {
    0, 0,
    0, 0, 
    0, 0,
    0, 0
};
double del_yaw2[4][2] = {
    -0.15, -0.15,
        0,     0,
        0,  0.15,
        0,     0
};
double x_2[4][2] = {
      0, -2.5,
    -15,  -15,
     15,  2.5,
     15,   15
};
double y_2[4][2] = {
      0,   0,
      0,   0,
      0,   0,
    -10, -10
};
double z_2[4][2] = {
    20, 20,
    12, 12,
    12, 20,
    12, 12
};
/* 第三阶段参数 */
double r = 4;
int point3 = 600;
double yaw3[4][2] = {
    0, 0,
    0, 0, 
    0, 0,
    0, 0
};
double del_yaw3[4] = {-0.15, 0, -0.15, -0.15};
double x_3[4][2] = {
    -2*sqrt(2),  pi*5/4,
           -12,      12,
            16,      -8,
            12,     -12
};
double y_3[4][2] = {
    -2*sqrt(2), -pi*3/2,
      4,       4,
      0,       0,
     -4,      -4
};
double z_3[4][2] = {
    10, 10,
     3,  3,
    10, 10,
    10, 10
};
/* 第四阶段参数 */
double z_end = 24;
int point4 = 300;
/* 初始位置点 */
double x_ini[4] = {0.0, -20.0, 20.0,  20.0};
double y_ini[4] = {0.0,   0.0,  0.0, -15.0};
/* 消息定义 */
geometry_msgs::PoseStamped pose[4];
nav_msgs::Odometry pose_odom[4];
mavros_msgs::State current_state;

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

geometry_msgs::Vector3 quaternion2euler(geometry_msgs::Quaternion quater);
geometry_msgs::Quaternion euler2quaternion(geometry_msgs::Vector3 euler);

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void plane_pos_cb0(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom[0] = *msg;
}
void plane_pos_cb1(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom[1] = *msg;
}
void plane_pos_cb2(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom[2] = *msg;
}
void plane_pos_cb3(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom[3] = *msg;
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

    ros::Rate rate(20.0);

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>解 锁 飞 机<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // wait for FCU connection
    while (ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    for (int i = 0; i < 4; ++i){
        geometry_msgs::Vector3 euler;
        euler.x = euler.y = euler.z = 0;
        pose[i].pose.orientation = euler2quaternion(euler);
        pose[i].pose.position.x = 0;
        pose[i].pose.position.y = 0;
        pose[i].pose.position.z = 12;
    }

    //send a few setpoints before starting
    for (int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose[0]);
        local_pos_pub1.publish(pose[1]);
        local_pos_pub2.publish(pose[2]);
        local_pos_pub3.publish(pose[3]);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    for (int i = 0; i < 400 && ros::ok(); ++i){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client0.call(offb_set_mode) &&
                set_mode_client1.call(offb_set_mode) &&
                set_mode_client2.call(offb_set_mode) &&
                set_mode_client3.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled : 4");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client0.call(arm_cmd) &&
                    arming_client1.call(arm_cmd) &&
                    arming_client2.call(arm_cmd) &&
                    arming_client3.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed : 4");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub0.publish(pose[0]);
        local_pos_pub1.publish(pose[1]);
        local_pos_pub2.publish(pose[2]);
        local_pos_pub3.publish(pose[3]);
        ros::spinOnce();
        rate.sleep();
    }
    
///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>第 一 阶 段<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    bool flag;
    for (int i = 0; i < point1; ++i){
        flag = false;
        while (ros::ok() && !flag) {
            for (int j = 0; j < 4; ++j){
                geometry_msgs::Vector3 euler;
                euler.x = euler.y = 0;
                euler.z = yaw1[j][i];
                pose[j].pose.orientation = euler2quaternion(euler);
                pose[j].pose.position.x = x_1[j][i] - x_ini[j];
                pose[j].pose.position.y = y_1[j][i] - y_ini[j];
                pose[j].pose.position.z = z_1[j][i];
                flag = ( ( fabs(pose[j].pose.position.x - pose_odom[j].pose.pose.position.x) < epsilon )
                 && ( fabs(pose[j].pose.position.y - pose_odom[j].pose.pose.position.y) < epsilon )
                 && ( fabs(pose[j].pose.position.z - pose_odom[j].pose.pose.position.z) < epsilon )
                 && ( fabs(euler.z - quaternion2euler(pose_odom[j].pose.pose.orientation).z) < epsilon ) );
            }
            local_pos_pub0.publish(pose[0]);
            local_pos_pub1.publish(pose[1]);
            local_pos_pub2.publish(pose[2]);
            local_pos_pub3.publish(pose[3]);
            ros::spinOnce();
            rate.sleep();
        }
        for (int count = 0; count < 300 && ros::ok(); ++count) {
            for (int j = 0; j < 4; ++j){
                geometry_msgs::Vector3 euler;
                euler.x = euler.y = 0;
                euler.z = yaw1[j][i];
                pose[j].pose.orientation = euler2quaternion(euler);
                pose[j].pose.position.x = x_1[j][i] - x_ini[j];
                pose[j].pose.position.y = y_1[j][i] - y_ini[j];
                pose[j].pose.position.z = z_1[j][i];
            }
            local_pos_pub0.publish(pose[0]);
            local_pos_pub1.publish(pose[1]);
            local_pos_pub2.publish(pose[2]);
            local_pos_pub3.publish(pose[3]);
            ros::spinOnce();
            rate.sleep();
        }
    }

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>第 二 阶 段<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    for (int i = 0; i < point2; ++i){
        flag = false;
        while (ros::ok() && !flag) {
            for (int j = 0; j < 4; ++j){
                geometry_msgs::Vector3 euler;
                euler.x = euler.y = 0;
                euler.z = yaw2[j][i];
                pose[j].pose.orientation = euler2quaternion(euler);
                pose[j].pose.position.x = x_2[j][i] - x_ini[j];
                pose[j].pose.position.y = y_2[j][i] - y_ini[j];
                pose[j].pose.position.z = z_2[j][i];
                flag = ( ( fabs(pose[j].pose.position.x - pose_odom[j].pose.pose.position.x) < epsilon )
                 && ( fabs(pose[j].pose.position.y - pose_odom[j].pose.pose.position.y) < epsilon )
                 && ( fabs(pose[j].pose.position.z - pose_odom[j].pose.pose.position.z) < epsilon )
                 && ( fabs(euler.z - quaternion2euler(pose_odom[j].pose.pose.orientation).z) < epsilon ) );
            }
            local_pos_pub0.publish(pose[0]);
            local_pos_pub1.publish(pose[1]);
            local_pos_pub2.publish(pose[2]);
            local_pos_pub3.publish(pose[3]);
            ros::spinOnce();
            rate.sleep();
        }
        for (int count = 0; count < 300 && ros::ok(); ++count) {
            for (int j = 0; j < 4; ++j){
                geometry_msgs::Vector3 euler;
                euler.x = euler.y = 0;
                euler.z = yaw2[j][i] + del_yaw2[j][i]*count;
                pose[j].pose.orientation = euler2quaternion(euler);
                pose[j].pose.position.x = x_2[j][i] - x_ini[j];
                pose[j].pose.position.y = y_2[j][i] - y_ini[j];
                pose[j].pose.position.z = z_2[j][i];
            }
            local_pos_pub0.publish(pose[0]);
            local_pos_pub1.publish(pose[1]);
            local_pos_pub2.publish(pose[2]);
            local_pos_pub3.publish(pose[3]);
            ros::spinOnce();
            rate.sleep();
        }
    }

    ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>第 三 阶 段<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    flag = false;
    int wait_loop = 0;
    while (ros::ok() && !flag && wait_loop < 100) {
        for (int i = 0; i < 4; ++i){
            geometry_msgs::Vector3 euler;
            euler.x = euler.y = 0;
            euler.z = yaw3[i][0];
            pose[i].pose.orientation = euler2quaternion(euler);
            pose[i].pose.position.x = x_3[i][0] - x_ini[i];
            pose[i].pose.position.y = y_3[i][0] - y_ini[i];
            pose[i].pose.position.z = z_3[i][0];
            flag = ( ( fabs(pose[i].pose.position.x - pose_odom[i].pose.pose.position.x) < epsilon )
                && ( fabs(pose[i].pose.position.y - pose_odom[i].pose.pose.position.y) < epsilon )
                && ( fabs(pose[i].pose.position.z - pose_odom[i].pose.pose.position.z) < epsilon )
                && ( fabs(euler.z - quaternion2euler(pose_odom[i].pose.pose.orientation).z) < epsilon ) );
            wait_loop += flag;
        }
        local_pos_pub0.publish(pose[0]);
        local_pos_pub1.publish(pose[1]);
        local_pos_pub2.publish(pose[2]);
        local_pos_pub3.publish(pose[3]);
        ros::spinOnce();
        rate.sleep();
    }
    for (int count = 0; count < point3 && ros::ok(); ++count) {
        for (int i = 0; i < 4; ++i){
            geometry_msgs::Vector3 euler;
            euler.x = euler.y = 0;
            euler.z = yaw3[i][0] + del_yaw3[i]*count;
            pose[i].pose.orientation = euler2quaternion(euler);
            pose[i].pose.position.x = (x_3[i][1] - x_3[i][0])/point3*count + x_3[i][0] - x_ini[i];
            pose[i].pose.position.y = (y_3[i][1] - y_3[i][0])/point3*count + y_3[i][0] - y_ini[i];
            pose[i].pose.position.z = (z_3[i][1] - z_3[i][0])/point3*count + z_3[i][0];
        }
        pose[0].pose.position.x = r*cos( (y_3[0][1] - x_3[0][1])/point3*count + x_3[0][1] ) - x_ini[0];
        pose[0].pose.position.y = r*sin( (y_3[0][1] - x_3[0][1])/point3*count + x_3[0][1] ) - y_ini[0];
        local_pos_pub0.publish(pose[0]);
        local_pos_pub1.publish(pose[1]);
        local_pos_pub2.publish(pose[2]);
        local_pos_pub3.publish(pose[3]);
        ros::spinOnce();
        rate.sleep();
    }

    ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>第 四 阶 段<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    flag = false;
    while (ros::ok() && !flag) {
        for (int i = 0; i < 4; ++i){
            geometry_msgs::Vector3 euler;
            euler.x = euler.y = euler.z = 0;
            pose[i].pose.orientation = euler2quaternion(euler);
            pose[i].pose.position.x = x_1[i][0] - x_ini[i];
            pose[i].pose.position.y = y_1[i][0] - y_ini[i];
            pose[i].pose.position.z = z_1[i][0];
            flag = ( ( fabs(pose[i].pose.position.x - pose_odom[i].pose.pose.position.x) < epsilon )
                && ( fabs(pose[i].pose.position.y - pose_odom[i].pose.pose.position.y) < epsilon )
                && ( fabs(pose[i].pose.position.z - pose_odom[i].pose.pose.position.z) < epsilon )
                && ( fabs(euler.z - quaternion2euler(pose_odom[i].pose.pose.orientation).z) < epsilon ) );
        }
        local_pos_pub0.publish(pose[0]);
        local_pos_pub1.publish(pose[1]);
        local_pos_pub2.publish(pose[2]);
        local_pos_pub3.publish(pose[3]);
        ros::spinOnce();
        rate.sleep();
    }
    for (int count = 0; count < point4 && ros::ok(); ++count) {
        for (int i = 1; i < 4; ++i){
            pose[i].pose.position.x = x_1[i][0] - x_ini[i];
            pose[i].pose.position.y = y_1[i][0] - y_ini[i];
            pose[i].pose.position.z = z_1[i][0];
        }
        geometry_msgs::Vector3 euler;
        euler.x = euler.y = euler.z = 0;
        pose[0].pose.orientation = euler2quaternion(euler);
        pose[0].pose.position.x = 0 - x_ini[0];
        pose[0].pose.position.y = 0 - y_ini[0];
        pose[0].pose.position.z = (z_end - z_1[0][0])/point4*count + z_1[0][0];
        local_pos_pub0.publish(pose[0]);
        local_pos_pub1.publish(pose[1]);
        local_pos_pub2.publish(pose[2]);
        local_pos_pub3.publish(pose[3]);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
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