//
// Created by timesong on 2021/12/22.
//
#include <ros/ros.h>
#include "pos_io/test_pub.h"
#include <iostream>
#include <string>

using namespace std;

void messageCallBack(const pos_io::test_pub::ConstPtr& msg){
    ROS_INFO("Success");
    ROS_INFO("Message received:%s,%d,%f,%f,%f,%f,%f,%f,%f,%s,%d,%f,%f,%f,%f,%f,%f,%f",msg->gtimu.c_str(),msg->imu_gps_week,msg->imu_gps_time,msg->imu_gyro_x,msg->imu_gyro_y,msg->imu_gyro_z,msg->imu_acc_x,msg->imu_acc_y,msg->imu_acc_z,msg->gpfpd.c_str(),msg->fpd_gps_week,msg->fpd_gps_time,msg->fpd_heading,msg->fpd_pitch,msg->fpd_roll,msg->fpd_lattitude,msg->fpd_longitude,msg->fpd_altitude);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"listen");
    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("pos_info",1000,messageCallBack);
    ros::spin();
    return 0;
}
