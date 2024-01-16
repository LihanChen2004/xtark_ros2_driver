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

#include "xtark_robot.hpp"

/*
 * @功  能  构造函数，机器人初始化
 */
XtarkRobot::XtarkRobot(const rclcpp::NodeOptions & options)
: Node("xtark_robot_node", options), tf_broadcaster_(this)
{
  // frame初始化
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("imu_frame", "imu_link");

  // 话题消息初始化
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("imu_topic", "imu");
  this->declare_parameter<std::string>("battery_topic", "bat_vol");
  this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

  //参数初始化
  this->get_parameter("robot_port", serial_port_);
  this->get_parameter("robot_port_baud", serial_port_baud_);
  this->get_parameter("pub_odom_tf", pub_odom_tf_);

  //仅AKM 机器人舵机零偏参数初始化
  this->get_parameter("akm_servo_offset", akm_servo_offset_);
  //模式选择
  this->get_parameter("light_mode", RGB_M1_);
  this->get_parameter("light_mode2", RGB_M2_);
  this->get_parameter("light_time", RGB_T_);
  this->get_parameter("rgb_r", RGB_R_);
  this->get_parameter("rgb_g", RGB_G_);
  this->get_parameter("rgb_b", RGB_B_);

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS());
  bat_pub_ = this->create_publisher<std_msgs::msg::Float32>("/bat", rclcpp::SensorDataQoS());

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 100, std::bind(&XtarkRobot::cmdVelCallback, this, std::placeholders::_1));

  light_sub_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
    "/light", 10, std::bind(&XtarkRobot::lightCallback, this, std::placeholders::_1));

  // 数据初始化
  memset(&pos_data_, 0, sizeof(pos_data_));
  last_msg_time_ = this->now();

  // 提示信息，串口端口号和波特率
  RCLCPP_INFO(
    this->get_logger(), "Xtark Robot Set serial %s at %d baud", serial_port_.c_str(),
    serial_port_baud_);

  // 初始化串口
  if (openSerialPort()) {
    try {
      //启动串口接收线程
      boost::thread recv_serial_thread(boost::bind(&XtarkRobot::recvCallback, this));
      // recvSerial_thread.join();
      RCLCPP_INFO(this->get_logger(), "Successfully launched recvSerial_thread.");
    } catch (...) {
      RCLCPP_INFO(this->get_logger(), "This is an info message");

      // 关闭节点
      rclcpp::shutdown();
    }
  } else {
    // 关闭节点
    rclcpp::shutdown();
  }

  RCLCPP_INFO(this->get_logger(), "Xtark Robot is Connected to OpenCRP board ");

  // 向阿克曼发送零位偏执修正角度，其它车型忽略
  static uint8_t offset_data[2];
  offset_data[0] = (int16_t)(akm_servo_offset_) >> 8;
  offset_data[1] = (int16_t)(akm_servo_offset_);
  sendSerialPacket(offset_data, 2, ID_ROS2CRP_AKM);
  RCLCPP_INFO(
    this->get_logger(), "Xtark Robot Set AKM Servo Offset Angle %d, other ignore ",
    akm_servo_offset_);

  // 发送灯光初始数据
  static uint8_t init_color[6];
  init_color[0] = RGB_M1_;
  init_color[1] = RGB_M2_;
  init_color[2] = RGB_T_;
  init_color[3] = RGB_R_;
  init_color[4] = RGB_G_;
  init_color[5] = RGB_B_;
  sendSerialPacket(init_color, 6, ID_ROS2CRP_LGT);
  RCLCPP_INFO(
    this->get_logger(), "Xtark Robot Set color_mode:%d, color_R:%d, color_G:%d, color_B:%d",
    RGB_M1_, RGB_R_, RGB_G_, RGB_B_);

  // 提示信息，发布订阅的话题消息
  RCLCPP_INFO(this->get_logger(), "Xtark Robot setup publisher  on odom [nav_msgs/Odometry]");
  RCLCPP_INFO(this->get_logger(), "Xtark Robot setup publisher  on imu [sensor_msgs/Imu]");
  RCLCPP_INFO(this->get_logger(), "Xtark Robot setup publisher  on bat_vol [float]");
  RCLCPP_INFO(this->get_logger(), "Xtark Robot setup subscriber on cmd_vel [geometry_msgs/Twist]");

  // 机器人启动完成提示信息
  RCLCPP_INFO(this->get_logger(), "Xtark Robot initialization completed, is Running!");

  // 创建底盘速度控制保护定时器（当没有接收到新速度消息时，串口发送给底盘的速度为0）
  timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {

    if ((this->now() - this->last_msg_time_).seconds() > 0.2) {
      // 如果距离上次接收到消息超过0.5秒，那么将 cmd_vel 速度全部置0
      uint8_t stop_data[6] = {0};
      this->sendSerialPacket(stop_data, 6, ID_ROS2CRP_VEL);
    }
  });
}

/*
 * @功  能  析构函数，对象生命周期结束时系统会调用这个函数
 */
XtarkRobot::~XtarkRobot()
{
  static uint8_t vel_data[11];

  //发送静止指令
  vel_data[0] = 0;
  vel_data[1] = 0;
  vel_data[2] = 0;
  vel_data[3] = 0;
  vel_data[4] = 0;
  vel_data[5] = 0;
  sendSerialPacket(vel_data, 6, ID_ROS2CRP_VEL);

  timer_->cancel();

  //关闭串口
  closeSerialPort();

  RCLCPP_INFO(this->get_logger(), "Xtark Robot shutting down.");
}

/*
 * @功  能  初始化串口
 */
bool XtarkRobot::openSerialPort()
{
  //检查串口是否已经被打开
  if (serial_ptr_) {
    RCLCPP_INFO(this->get_logger(), "The SerialPort is already opened!\r\n");
    return false;
  }

  //打开串口
  serial_ptr_ = serial_ptr(new boost::asio::serial_port(io_service_));
  serial_ptr_->open(serial_port_, err_code_);

  //串口是否正常打开
  if (err_code_) {
    RCLCPP_INFO(this->get_logger(), "Open Port: %s Failed! Aboart!", serial_port_.c_str());
    return false;
  }

  //初始化串口参数
  serial_ptr_->set_option(boost::asio::serial_port_base::baud_rate(serial_port_baud_));
  serial_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
  serial_ptr_->set_option(
    boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  serial_ptr_->set_option(
    boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  serial_ptr_->set_option(
    boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  return true;
}

/*
 * @功  能  关闭串口
 */
void XtarkRobot::closeSerialPort()
{
  //如果串口被打开，则关闭串口
  if (serial_ptr_) {
    serial_ptr_->cancel();
    serial_ptr_->close();
    serial_ptr_.reset();
  }

  io_service_.stop();
  io_service_.reset();
}

void XtarkRobot::lightCallback(const std::shared_ptr<const std_msgs::msg::ColorRGBA> & light)
{
  setColor(light->r, light->g, light->b, light->a, RGB_M1_);
}

//协议解析变量
uint8_t RX_CON = 0;   //接收计数器
uint8_t RX_CHECKSUM;  //帧头部分校验和
uint8_t RX_BUF[60];   //接收缓冲
/*
 * @功  能  串口接收回调函数，接收塔克X-Protocol
 */
void XtarkRobot::recvCallback()
{
  //接收数据
  uint8_t res;

  while (1) {
    //读取串口数据
    boost::asio::read(*serial_ptr_, boost::asio::buffer(&res, 1), err_code_);

    //塔克X-Protocol协议解析数据
    if (RX_CON < 3)  //=========接收帧头 + 长度
    {
      if (RX_CON == 0)  //接收帧头1 0xAA
      {
        if (res == 0xAA) {
          RX_BUF[0] = res;
          RX_CON = 1;
        } else {
        }
      } else if (RX_CON == 1)  //接收帧头2 0x55
      {
        if (res == 0x55) {
          RX_BUF[1] = res;
          RX_CON = 2;
        } else {
          RX_CON = 0;
        }
      } else  //接收数据长度
      {
        RX_BUF[2] = res;
        RX_CON = 3;
        RX_CHECKSUM = (0xAA + 0x55) + res;  //计算校验和
      }
    } else  //=========接收数据
    {
      if (RX_CON < (RX_BUF[2] - 1)) {
        RX_BUF[RX_CON] = res;
        RX_CON++;
        RX_CHECKSUM = RX_CHECKSUM + res;
      } else  //判断最后1位
      {
        //接收完成，恢复初始状态
        RX_CON = 0;

        //数据校验
        if (res == RX_CHECKSUM)  //校验正确
        {
          //调用串口数据处理函数
          recvDataHandle(RX_BUF);
        }
      }
    }
  }
}

/*
 * @功  能  串口接收数据处理
 */
void XtarkRobot::recvDataHandle(uint8_t * buffer_data)
{
  //机器人数据
  if (buffer_data[3] == ID_CPR2ROS_DATA) {
    //解析IMU加速度数据
    imu_data_.acc_x = ((double)((int16_t)(buffer_data[4] * 256 + buffer_data[5])) * ACC_RATIO);
    imu_data_.acc_y = ((double)((int16_t)(buffer_data[6] * 256 + buffer_data[7])) * ACC_RATIO);
    imu_data_.acc_z = ((double)((int16_t)(buffer_data[8] * 256 + buffer_data[9])) * ACC_RATIO);

    //解析IMU陀螺仪数据
    imu_data_.gyro_x = ((double)((int16_t)(buffer_data[10] * 256 + buffer_data[11])) * GYRO_RATIO);
    imu_data_.gyro_y = ((double)((int16_t)(buffer_data[12] * 256 + buffer_data[13])) * GYRO_RATIO);
    imu_data_.gyro_z = ((double)((int16_t)(buffer_data[14] * 256 + buffer_data[15])) * GYRO_RATIO);

    //解析机器人速度
    vel_data_.linear_x = ((double)((int16_t)(buffer_data[16] * 256 + buffer_data[17])) / 1000);
    vel_data_.linear_y = ((double)((int16_t)(buffer_data[18] * 256 + buffer_data[19])) / 1000);
    vel_data_.angular_z = ((double)((int16_t)(buffer_data[20] * 256 + buffer_data[21])) / 1000);

    //解析电压值
    bat_vol_data_ = (double)(((buffer_data[22] << 8) + buffer_data[23])) / 100;

    //计算里程计数据
    pos_data_.pos_x += (vel_data_.linear_x * cos(pos_data_.angular_z) -
                        vel_data_.linear_y * sin(pos_data_.angular_z)) *
                       DATA_PERIOD;
    pos_data_.pos_y += (vel_data_.linear_x * sin(pos_data_.angular_z) +
                        vel_data_.linear_y * cos(pos_data_.angular_z)) *
                       DATA_PERIOD;
    pos_data_.angular_z += vel_data_.angular_z * DATA_PERIOD;  //绕Z轴的角位移，单位：rad

    //计算IMU四元数数据
    calculateImuQuaternion(imu_data_);

    //发布话题消息
    publishOdom();    //发布里程计话题
    publishImu();     //发布IMU传感器话题
    publishBatVol();  //发布电池电压话题

    //发布odom里程计TF坐标变换（odom --> base_footprint）
    publishOdomTF();
  }
}

/*
 * @功  能  发布IMU话题消息
 */
void XtarkRobot::publishImu()
{
  //获取数据
  imu_msgs_.header.stamp = this->now();
  imu_msgs_.header.frame_id = imu_frame_;
  imu_msgs_.angular_velocity.x = imu_data_.gyro_x;
  imu_msgs_.angular_velocity.y = imu_data_.gyro_y;
  imu_msgs_.angular_velocity.z = imu_data_.gyro_z;
  imu_msgs_.linear_acceleration.x = imu_data_.acc_x;
  imu_msgs_.linear_acceleration.y = imu_data_.acc_y;
  imu_msgs_.linear_acceleration.z = imu_data_.acc_z;
  imu_msgs_.orientation.x = orient_data_.x;
  imu_msgs_.orientation.y = orient_data_.y;
  imu_msgs_.orientation.z = orient_data_.z;
  imu_msgs_.orientation.w = orient_data_.w;

  //不使用姿态角度
  imu_msgs_.orientation.x = 0;
  imu_msgs_.orientation.y = 0;

  //协防差矩阵
  imu_msgs_.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
  imu_msgs_.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
  imu_msgs_.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  //发布
  imu_pub_->publish(imu_msgs_);
}

/*
 * @功  能  发布odom里程计话题消息
 */
void XtarkRobot::publishOdom()
{
  //计算里程计四元数
  tf2::Quaternion odom_quat;
  odom_quat.setRPY(0, 0, pos_data_.angular_z);

  //获取数据
  odom_msgs_.header.stamp = this->now();
  odom_msgs_.header.frame_id = odom_frame_;
  odom_msgs_.child_frame_id = base_frame_;
  odom_msgs_.pose.pose.position.x = pos_data_.pos_x;
  odom_msgs_.pose.pose.position.y = pos_data_.pos_y;
  odom_msgs_.pose.pose.position.z = 0;  //高度为0
  odom_msgs_.pose.pose.orientation.x = odom_quat.getX();
  odom_msgs_.pose.pose.orientation.y = odom_quat.getY();
  odom_msgs_.pose.pose.orientation.z = odom_quat.getZ();
  odom_msgs_.pose.pose.orientation.w = odom_quat.getW();
  odom_msgs_.twist.twist.linear.x = vel_data_.linear_x;
  odom_msgs_.twist.twist.linear.y = vel_data_.linear_y;
  odom_msgs_.twist.twist.angular.z = vel_data_.angular_z;

  //里程计协防差矩阵，用于robt_pose_ekf功能包，静止和运动使用不同的参数
  if (vel_data_.linear_x == 0 && vel_data_.linear_y == 0 && vel_data_.angular_z == 0) {
    //机器人静止时，IMU水平陀螺仪会存在零飘，编码器没有误差，编码器数据权重增加
    odom_msgs_.pose.covariance = {1e-9, 0, 0,   0, 0,   0, 0, 1e-3, 1e-9, 0,   0, 0,
                                  0,    0, 1e6, 0, 0,   0, 0, 0,    0,    1e6, 0, 0,
                                  0,    0, 0,   0, 1e6, 0, 0, 0,    0,    0,   0, 1e-9};

    odom_msgs_.twist.covariance = {1e-9, 0, 0,   0, 0,   0, 0, 1e-3, 1e-9, 0,   0, 0,
                                   0,    0, 1e6, 0, 0,   0, 0, 0,    0,    1e6, 0, 0,
                                   0,    0, 0,   0, 1e6, 0, 0, 0,    0,    0,   0, 1e-9};
  } else {
    //机器人运动时，轮子滑动编码器误差增加，IMU陀螺仪数据更加准确，IMU数据权重增加
    odom_msgs_.pose.covariance = {1e-3, 0, 0,   0, 0,   0, 0, 1e-3, 0, 0,   0, 0,
                                  0,    0, 1e6, 0, 0,   0, 0, 0,    0, 1e6, 0, 0,
                                  0,    0, 0,   0, 1e6, 0, 0, 0,    0, 0,   0, 1e3};

    odom_msgs_.twist.covariance = {1e-3, 0, 0,   0, 0,   0, 0, 1e-3, 0, 0,   0, 0,
                                   0,    0, 1e6, 0, 0,   0, 0, 0,    0, 1e6, 0, 0,
                                   0,    0, 0,   0, 1e6, 0, 0, 0,    0, 0,   0, 1e3};
  }

  //发布
  odom_pub_->publish(odom_msgs_);
}

/*
 * @功  能  发布电池电压话题消息
 */
void XtarkRobot::publishBatVol()
{
  //获取数据
  bat_msgs_.data = bat_vol_data_;

  //发布
  bat_pub_->publish(bat_msgs_);
}

/*
 * @功  能  发布里程计到base_footprint的TF坐标变换
 */
void XtarkRobot::publishOdomTF()
{
  //发布里程计到footprint坐标变换
  if (pub_odom_tf_) {
    //计算里程计TF四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, pos_data_.angular_z);

    //填充数据
    transform_stamped_.header.stamp = this->now();
    transform_stamped_.header.frame_id = odom_frame_;
    transform_stamped_.child_frame_id = base_frame_;

    transform_stamped_.transform.translation.x = pos_data_.pos_x;
    transform_stamped_.transform.translation.y = pos_data_.pos_y;
    transform_stamped_.transform.translation.z = 0.0;

    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();

    // 发布TF坐标变换
    tf_broadcaster_.sendTransform(transform_stamped_);
  }
}

/*
 * @功  能  cmd_vel 速度话题订阅回调函数，根据订阅的指令通过串口发指令控制下位机
 */
void XtarkRobot::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_msg_time_ = this->now();

  static uint8_t vel_data[11];

  //数据转换
  vel_data[0] = (int16_t)(msg->linear.x * 1000) >> 8;
  vel_data[1] = (int16_t)(msg->linear.x * 1000);
  vel_data[2] = (int16_t)(msg->linear.y * 1000) >> 8;
  vel_data[3] = (int16_t)(msg->linear.y * 1000);
  vel_data[4] = (int16_t)(msg->angular.z * 1000) >> 8;
  vel_data[5] = (int16_t)(msg->angular.z * 1000);

  //发送串口数据
  sendSerialPacket(vel_data, 6, ID_ROS2CRP_VEL);
}

/*
 * @功  能  light 灯光话题订阅回调函数，根据订阅的指令通过串口发指令控制下位机
 */
void XtarkRobot::setColor(float r, float g, float b, float a, uint8_t selet)
{
  static uint8_t send_buff[11];

  //数据转换
  send_buff[0] = selet;
  send_buff[1] = 0;
  send_buff[2] = 0;
  send_buff[3] = (uint8_t)(r * a * 255);
  send_buff[4] = (uint8_t)(g * a * 255);
  send_buff[5] = (uint8_t)(b * a * 255);

  //发送串口数据
  sendSerialPacket(send_buff, 6, ID_ROS2CRP_LGT);
  RCLCPP_INFO(
    this->get_logger(), "Light Control : selet: %d R: %d, G: %d, B: %d", selet, send_buff[6],
    send_buff[7], send_buff[8]);
}

/*
 * @功  能  发送串口数据包，塔克X-Protocol协议
 * @参  数  *pbuf：发送数据指针
 *          len：发送数据长度个数，长度小于64字节
 *          num：帧号，帧编码
 * @返回值	 无
 */
void XtarkRobot::sendSerialPacket(uint8_t * pbuf, uint8_t len, uint8_t num)
{
  uint8_t i, cnt;
  uint8_t tx_checksum = 0;  //发送校验和
  uint8_t tx_buf[64];

  //判断是否超出长度
  if (len <= 64) {
    //获取数据
    tx_buf[0] = 0xAA;     //帧头
    tx_buf[1] = 0x55;     //
    tx_buf[2] = len + 5;  //根据输出长度计算帧长度
    tx_buf[3] = num;      //帧编码

    for (i = 0; i < len; i++) {
      tx_buf[4 + i] = *(pbuf + i);
    }

    //计算校验和
    cnt = 4 + len;
    for (i = 0; i < cnt; i++) {
      tx_checksum = tx_checksum + tx_buf[i];
    }
    tx_buf[i] = tx_checksum;

    //计算帧长度
    cnt = len + 5;

    //发送数据
    boost::asio::write(*serial_ptr_, boost::asio::buffer(tx_buf, cnt), err_code_);
  }
}

/***************四元数计算**************************************************/
volatile float TWO_KP = 1.0f;  // 2 * proportional gain (Kp)
volatile float TWO_KI = 0.0f;  // 2 * integral gain (Ki)
                               // quaternion of sensor frame relative to auxiliary frame
volatile float Q0 = 1.0f, Q1 = 0.0f, Q2 = 0.0f, Q3 = 0.0f;
// integral error terms scaled by Ki
volatile float INTEGRAL_F_BX = 0.0f, INTEGRAL_F_BY = 0.0f, INTEGRAL_F_BZ = 0.0f;
volatile const float SAMPLING_PERIOD = DATA_PERIOD;

float invSqrt(float number);

/*
 * @功  能  计算IMU四元数
 */
void XtarkRobot::calculateImuQuaternion(struct ImuData imu_cel)
{
  float recip_norm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  float roll, pitch, yaw;

  //首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
  recip_norm = invSqrt(
    imu_cel.acc_x * imu_cel.acc_x + imu_cel.acc_y * imu_cel.acc_y + imu_cel.acc_z * imu_cel.acc_z);

  imu_cel.acc_x *= recip_norm;
  imu_cel.acc_y *= recip_norm;
  imu_cel.acc_z *= recip_norm;

  // 把四元数换算成方向余弦中的第三行的三个元素
  halfvx = Q1 * Q3 - Q0 * Q2;
  halfvy = Q0 * Q1 + Q2 * Q3;
  halfvz = Q0 * Q0 - 0.5f + Q3 * Q3;

  //误差是估计的重力方向和测量的重力方向的交叉乘积之和
  halfex = (imu_cel.acc_y * halfvz - imu_cel.acc_z * halfvy);
  halfey = (imu_cel.acc_z * halfvx - imu_cel.acc_x * halfvz);
  halfez = (imu_cel.acc_x * halfvy - imu_cel.acc_y * halfvx);

  // 计算并应用积分反馈（如果启用）
  if (TWO_KI > 0.0f) {
    INTEGRAL_F_BX += TWO_KI * halfex * SAMPLING_PERIOD;  // integral error scaled by Ki
    INTEGRAL_F_BY += TWO_KI * halfey * SAMPLING_PERIOD;
    INTEGRAL_F_BZ += TWO_KI * halfez * SAMPLING_PERIOD;
    imu_cel.gyro_x += INTEGRAL_F_BX;  // apply integral feedback
    imu_cel.gyro_y += INTEGRAL_F_BY;
    imu_cel.gyro_z += INTEGRAL_F_BZ;
  } else {
    INTEGRAL_F_BX = 0.0f;  // prevent integral windup
    INTEGRAL_F_BY = 0.0f;
    INTEGRAL_F_BZ = 0.0f;
  }
  // Apply proportional feedback
  imu_cel.gyro_x += TWO_KP * halfex;
  imu_cel.gyro_y += TWO_KP * halfey;
  imu_cel.gyro_z += TWO_KP * halfez;

  // Integrate rate of change of quaternion
  imu_cel.gyro_x *= (0.5f * SAMPLING_PERIOD);  // pre-multiply common factors
  imu_cel.gyro_y *= (0.5f * SAMPLING_PERIOD);
  imu_cel.gyro_z *= (0.5f * SAMPLING_PERIOD);

  qa = Q0;
  qb = Q1;
  qc = Q2;

  Q0 += (-qb * imu_cel.gyro_x - qc * imu_cel.gyro_y - Q3 * imu_cel.gyro_z);
  Q1 += (qa * imu_cel.gyro_x + qc * imu_cel.gyro_z - Q3 * imu_cel.gyro_y);
  Q2 += (qa * imu_cel.gyro_y - qb * imu_cel.gyro_z + Q3 * imu_cel.gyro_x);
  Q3 += (qa * imu_cel.gyro_z + qb * imu_cel.gyro_y - qc * imu_cel.gyro_x);

  // Normalise quaternion
  recip_norm = invSqrt(Q0 * Q0 + Q1 * Q1 + Q2 * Q2 + Q3 * Q3);

  Q0 *= recip_norm;
  Q1 *= recip_norm;
  Q2 *= recip_norm;
  Q3 *= recip_norm;

  //计算结果赋值到
  orient_data_.w = Q0;
  orient_data_.x = Q1;
  orient_data_.y = Q2;
  orient_data_.z = Q3;

  //计算欧拉角
  // roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
  // pitch = asinf(-2.0f * (q1*q3 - q0*q2));
  // yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
  // ROS_INFO("IMU:%f  %f  %f ",roll,pitch,yaw);
}

/*
 * @功  能  平方根倒数 求四元数
 */
float invSqrt(float x)
{
  volatile long i;
  volatile float halfx, y;
  volatile const float f = 1.5F;

  halfx = x * 0.5F;
  y = x;
  i = *((long *)&y);

  i = 0x5f375a86 - (i >> 1);
  y = *((float *)&i);
  y = y * (f - (halfx * y * y));

  return y;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(XtarkRobot)