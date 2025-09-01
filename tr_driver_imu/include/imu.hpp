#ifndef IMU_HPP
#define IMU_HPP

#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <math.h>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "std_srvs/srv/trigger.hpp"

#include <libserial/SerialPort.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <lowPassFilter.hpp>
#include <extendedKalmanFilter.hpp>

#include <nlohmann/json.hpp>

#include <Eigen/Dense>

using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace std::literals;
using nlohmann::json;
// using namespace Eigen;

#define SAMPLE_IMU_TIMES 500

typedef struct
{
    // Acceleration
    float a_x;
    float a_y;
    float a_z;

    // Angular velocity
    float g_x;
    float g_y;
    float g_z;

    // Euler
    float e_yaw;
    float e_pitch;
    float e_roll;

    // Magnetic
    float m_x;
    float m_y;
    float m_z;

} ImuAhrs;
// ASCII CODE
const unsigned char A = 0x61;
const unsigned char C = 0x63;
const unsigned char D = 0x64;
const unsigned char F = 0x66;
const unsigned char G = 0x67;
const unsigned char M = 0x6D;
const unsigned char N = 0x6E;
const unsigned char P = 0x70;
const unsigned char R = 0x72;
const unsigned char S = 0x73;
const unsigned char Y = 0x79;
const unsigned char CR = 0x0D;
const unsigned char LF = 0x0A;

const char eol('\n');
const size_t timeout(1000);

class Imu : public rclcpp::Node
{
public:
    Imu();
    ~Imu();

private:
    int imu_device_ = -1;
    ImuAhrs ahrs_obj_ = {};
    float yaw_offset_ = 0.0;
    bool first_flag_ = true;

    std::string imu_topic_ = "imu";
    bool calibrate_imu_ = false;
    double imu_freq_ = 100.0;
    int init_calibration_cnt_ = 0;  // Keep track of how many samples have been taken
    bool calibration_start_ = true; // Start calibration flag

    // imu calibration bias
    float acc_x_bias = 0.0;
    float acc_y_bias = 0.0;
    float acc_z_bias = 0.0;
    float vel_theta_bias = 0;

    std::shared_ptr<LibSerial::SerialPort> serial_;
    /// Serial parameter
    std::string port_ = "/dev/ttyS1";
    // Thread for reading message
    std::shared_ptr<std::thread> read_thread_;
    // filter
    std::shared_ptr<LowPassFilter> vel_filter_; // Default filter with natural frequency = 800 Hz, damping = 0.707
    std::shared_ptr<LowPassFilter> acc_filter_; // Another filter with custom frequency and damping

    std::shared_ptr<ExtendedKalmanFilter> vel_ekf_filter_;
    std::shared_ptr<ExtendedKalmanFilter> acc_ekf_filter_;
    std::shared_ptr<ExtendedKalmanFilter> yaw_ekf_filter_;

private:
    void declareParameters();
    void getParameters();
    void initSerial();
    void read();
    void start();
    void imuPublishingTimer();
    double normalizeAngle(double angle);

    void reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

public:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
    /*Timer*/
    rclcpp::TimerBase::SharedPtr imu_pub_timer_;
    rclcpp::TimerBase::SharedPtr imu_data_timer_;
};

#endif