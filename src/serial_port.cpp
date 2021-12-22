//
// Created by timesong on 2021/12/21.
//
//serial_port.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>

#include "pos_io/test_pub.h"
using namespace std;

void pub(ros::Publisher* pub1);
bool checkWrongChar(std::string checked_string);
bool checkWrongNum(std::string checked_string);
std::mutex mut;
vector<char> big_buffer;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

    ros::Publisher test_pub=n.advertise<pos_io::test_pub>("pos_info",1000);
    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(460800);
    //串口设置timeout
    sp.setTimeout(to);

    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    std::thread th1(pub,&test_pub);
    th1.detach();

    ros::Rate loop_rate(100);

//    double test=stoi("K1968ܗ");


    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            std::lock_guard<std::mutex> guard(mut);   //上锁

            uint8_t* buffer=new uint8_t[n];
            //读出数据
            //n = sp.read(buffer, n);
            sp.read(buffer,n);
            for(int i=0; i<n; i++)
            {
                //std::cout<<buffer[i];
                big_buffer.push_back(buffer[i]);
                //cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;
                //16进制的方式打印到屏幕
                //std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            //std::cout << std::endl;
            //把数据发送回去
            //sp.write(buffer, n);
            delete[] buffer;
        }
        loop_rate.sleep();
    }

    //关闭串口
    sp.close();

    return 0;
}

void pub(ros::Publisher* pub1){
    ros::Rate loop_rate(100);
    while(ros::ok()){
        //cout<<"******************************************************************************************"<<endl;
        if(big_buffer.size()!=0){
            // auto index=
            std::lock_guard<std::mutex> guard(mut);   //上锁
            if(std::find(big_buffer.begin(),big_buffer.end(),'$')!=big_buffer.end()){
                if(big_buffer[0]!='$'){
                     auto index=std::find(big_buffer.begin(),big_buffer.end(),'$');
                     big_buffer.erase(big_buffer.begin(),index);   //删除多余元素
                 }
                if(std::find(big_buffer.begin(),big_buffer.end(),'\n')!=big_buffer.end()){
                    pos_io::test_pub msg;
                    auto ite=std::find(big_buffer.begin(),big_buffer.end(),'\n');
                    string message(big_buffer.begin(),ite+1);
                    cout<<"******************************************************************************************"<<endl;
                    if(checkWrongChar(message)){
                        stringstream input_string(message);
                        vector<string> data;
                        string buffer;
                        cout<<"----------------------------------------------------------------------------------------"<<endl;
                        //$GTIMU
                        if(big_buffer[1]=='G'&&big_buffer[2]=='T'&&big_buffer[3]=='I'&&big_buffer[4]=='M'&&big_buffer[5]=='U'){
                            cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
                            while (getline(input_string,buffer,',')){
                                data.push_back(buffer);
                            }
                            if(data.size()==10&& checkWrongNum(data[1])&&checkWrongNum(data[2])&&checkWrongNum(data[3])&&checkWrongNum(data[4])&&checkWrongNum(data[5])&&checkWrongNum(data[6])&&checkWrongNum(data[7])&&checkWrongNum(data[8])){
                                msg.gtimu=data[0];
                                msg.imu_gps_week=stoi(data[1]);
                                msg.imu_gps_time=stold(data[2]);
                                msg.imu_gyro_x=stold(data[3]);
                                msg.imu_gyro_y=stold(data[4]);
                                msg.imu_gyro_z=stold(data[5]);
                                msg.imu_acc_x=stold(data[6]);
                                msg.imu_acc_y=stold(data[7]);
                                msg.imu_acc_z=stold(data[8]);
                                cout<<msg.gtimu<<","<<msg.imu_gps_week<<","<<msg.imu_gps_time<<","<<msg.imu_gyro_x<<","<<msg.imu_gyro_y<<","<<msg.imu_gyro_z<<","<<msg.imu_acc_x<<","<<msg.imu_acc_y<<","<<msg.imu_acc_z<<endl;
                                // big_buffer.erase(big_buffer.begin(),ite+1);
                                pub1->publish(msg);
                            }
                            //ros::spinOnce();
                        }
                        //$GPFPD
                        else if(big_buffer[1]=='G'&&big_buffer[2]=='P'&&big_buffer[3]=='F'&&big_buffer[4]=='P'&&big_buffer[5]=='D'){
                            // cout<<"#########################################################################################"<<endl;
                            while (getline(input_string,buffer,',')){
                                data.push_back(buffer);
                            }
                            if(data.size()==16&& checkWrongNum(data[1])&&checkWrongNum(data[2])&&checkWrongNum(data[3])&&checkWrongNum(data[4])&&checkWrongNum(data[5])&&checkWrongNum(data[6])&&checkWrongNum(data[7])&&checkWrongNum(data[8])){
                                msg.gpfpd=data[0];
                                msg.fpd_gps_week=stoi(data[1]);
                                msg.fpd_gps_time=stold(data[2]);
                                msg.fpd_heading=stold(data[3]);
                                msg.fpd_pitch=stold(data[4]);
                                msg.fpd_roll=stold(data[5]);
                                msg.fpd_lattitude=stold(data[6]);
                                msg.fpd_longitude=stold(data[7]);
                                msg.fpd_altitude=stold(data[8]);
                                cout<<msg.gpfpd<<","<<msg.fpd_gps_week<<","<<msg.fpd_gps_time<<","<<msg.fpd_heading<<","<<msg.fpd_pitch<<","<<msg.fpd_roll<<","<<msg.fpd_lattitude<<","<<msg.fpd_longitude<<","<<msg.fpd_altitude<<endl;

                                pub1->publish(msg);

                            }
                            //ros::spinOnce();
                        }
                    }
                    big_buffer.erase(big_buffer.begin(),ite+1);
                }
            }
        }
        loop_rate.sleep();
    }

}
bool checkWrongChar(std::string checked_string){
    for(int i=0;i<checked_string.length();i++){
        char test=checked_string[i];
        if(isalnum(checked_string[i])) continue;
        else if(checked_string[i]=='$'||checked_string[i]=='-'||checked_string[i]=='.'||checked_string[i]==','||checked_string[i]=='*'||checked_string[i]=='\r'||checked_string[i]=='\n') continue;
        else {return false;}
    }
    return true;
}

bool checkWrongNum(std::string checked_string){
    for(int i=0;i<checked_string.length();i++){
        if(isdigit(checked_string[i])) continue;
        else if(checked_string[i]=='.'||checked_string[i]=='-') continue;
        else return false;
    }
    return true;
}