/*
 * Copyright 2022 XTARK ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef XTARK_ROBOT_H
#define XTARK_ROBOT_H

//C语言相关头文件
#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <termios.h>

//Boost库文件
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

//ROS相关头文件
// ROS1
// #include <ros/ros.h>    
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/Int16.h>
// #include <std_msgs/UInt16.h>
// #include <std_msgs/Float32.h>
// #include <sensor_msgs/Range.h>
// #include <sensor_msgs/Imu.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <std_msgs/ColorRGBA.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>

// #include <dynamic_reconfigure/server.h> //???
// #include <xtark_robot/robotConfig.h> //???

#define PI 3.1415926

//机器人通信帧头定义
#define  ID_CPR2ROS_DATA    0x10    //下位计向ROS发送的综合数据
#define  ID_ROS2CRP_VEL     0x50    //ROS向下位机发送的速度数据
#define  ID_ROS2CRP_IMU     0x51    //ROS向下位机发送的IMU陀螺仪零偏校准指令
#define  ID_ROS2CRP_AKM     0x5f    //ROS向下位机发送的AKM机器人舵机零偏补偿角，其它类型忽略
#define  ID_ROS2CRP_LGT     0x52    //ROS向下位机发送的速度数据

#define  LIGHT_M1    0x10    //单色模式  
#define  LIGHT_M2    0x20    //呼吸模式  

//IMU加速度计量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACC_RATIO 	  (2*9.8/32768)

//IMU陀螺仪量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位
#define GYRO_RATIO   ((500*PI/180)/32768)

//机器人数据处理周期,单位S
#define DATA_PERIOD   0.02f

typedef boost::shared_ptr<boost::asio::serial_port> serial_ptr;

//IMU数据结构体
struct imu_data
{
    float acc_x;
    float acc_y;
    float acc_z;    

    float gyro_x;
    float gyro_y;
    float gyro_z;
};

//IMU四元数结构体
struct imu_orientation_data
{
    float w;
    float x;
    float y;
    float z;    
};

//机器人速度数据结构体
struct velocity_data
{
    float linear_x;
    float linear_y;
    float angular_z;    
};

//机器人位置数据结构体
struct pose_data
{
    float pos_x;
    float pos_y;
    float angular_z;    
};


//机器人底盘驱动类
class XtarkRobot : public rclcpp::Node
{
    public:
        // XtarkRobot(const rclcpp::NodeOptions& options); //构造函数
        explicit XtarkRobot(const rclcpp::NodeOptions & options); //构造函数
        // ~XtarkRobot();  //析构函数
        ~XtarkRobot() override; //析构函数

    private:
        //串口操作
        bool openSerialPort();
        void closeSerialPort();

        //串口发送数据，塔克X-Protocol协议
        void sendSerialPacket(uint8_t *pbuf, uint8_t len, uint8_t num);
        
        //串口多线程接收函数
        void recvCallback();
        void recvDataHandle(uint8_t* buffer_data);

        //ROS1
        //速度消息回调函数
        // void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        // //灯光消息回调函数
        // void light_callback(const std_msgs::ColorRGBA::ConstPtr& light);
        // void SetColor(float r, float g, float b, float a, uint8_t selet); //selet: 0X10  单色模式   0x20    呼吸模式
        // //动态参数配置回调函数
        // void dynamicReconfigCallback(xtark_robot::robotConfig &config, uint32_t level);

        //ROS2
        //速度消息回调函数
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        //灯光消息回调函数
        void light_callback(const std::shared_ptr<const std_msgs::msg::ColorRGBA>& light);
        void SetColor(float r, float g, float b, float a, uint8_t selet); //selet: 0X10  单色模式   0x20    呼吸模式
        //动态参数配置回调函数
        // void dynamicReconfigCallback(xtark_robot::robotConfig &config, uint32_t level);

        //发布话题消息
        void publishOdom();     //发布里程计话题
		void publishImu();      //发布IMU传感器话题
		void publishBatVol();   //发布电池电压话题

        //发布odom里程计TF坐标变换
        void publishOdomTF();   

        //计算IMU 四元数
        void calculateImuQuaternion(struct imu_data imu_cel);

        //串口指针
        std::string     serial_port_ = "/dev/ttyACM0";      
        int             serial_port_baud_ = 115200;
        boost::shared_ptr<boost::asio::serial_port>  serial_ptr_;
        boost::system::error_code err_code_;
        boost::asio::io_service io_service_;

        //数据定义
        struct imu_data imu_data_;    //IMU数据
        struct imu_orientation_data orient_data_;  //IMU四元数姿态数据
        struct velocity_data vel_data_;    //机器人的速度
        struct pose_data pos_data_;    //机器人的位置
        float  bat_vol_data_;    //机器人电池电压数据

        //AKM机器人专用，舵机偏置补偿角度，其它机器人忽略
        int     akm_servo_offset_;   
        //灯光参数
        int     RGB_M1_; //灯光模式
        int     RGB_M2_; //暂留
        int     RGB_T_;  //暂留
        int     RGB_R_;  //红光占比
        int     RGB_G_;  //绿光占比
        int     RGB_B_;  //蓝光占比

        //Frame定义
        std::string odom_frame_;    //里程计
        std::string base_frame_;  //机器人
        std::string imu_frame_;     //IMU

        //话题定义
        std::string odom_topic_;    //里程计
        std::string imu_topic_;     //IMU话题
        std::string bat_topic_;     //电池话题
        std::string cmd_vel_topic_;     //电池话题

        //消息定义
        //ROS1
        // nav_msgs::Odometry odom_msgs_;  //里程计发布消息
        // sensor_msgs::Imu imu_msgs_;  //IMU发布消息
        // std_msgs::Float32 bat_msgs_;  //电池电压发布消息

        //ROS2
        nav_msgs::msg::Odometry odom_msgs_;  //里程计发布消息
        sensor_msgs::msg::Imu imu_msgs_;  //IMU发布消息
        std_msgs::msg::Float32 bat_msgs_;  //电池电压发布消息

        //发布器定义
        //ROS1
        // ros::Publisher odom_pub_;  
        // ros::Publisher bat_pub_;
        // ros::Publisher imu_pub_;

        //ROS2
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bat_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

        //订阅器定义
        //ROS1
        // ros::Subscriber cmd_vel_sub_;
        // ros::Subscriber light_sub_;

        //ROS2
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr light_sub_;

        //TF变换定义
        //ROS1
        // bool pub_odom_tf_;
        // geometry_msgs::TransformStamped transform_stamped_;
        // tf2_ros::TransformBroadcaster transform_broadcaster_;

        //ROS2
        bool pub_odom_tf_;
        geometry_msgs::msg::TransformStamped transform_stamped_;
        // 创建一个TransformBroadcaster对象
        tf2_ros::TransformBroadcaster tf_broadcaster_;
};

#endif
