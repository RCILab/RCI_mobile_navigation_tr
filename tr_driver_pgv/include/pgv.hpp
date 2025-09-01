#ifndef PGV_HPP
#define PGV_HPP

#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <math.h>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <bitset>
#include <signal.h>
#include <sstream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "tr_driver_msgs/msg/pgv_data.hpp"
#include "tr_driver_msgs/msg/pgv_dir_cmd.hpp"
#include "tr_driver_msgs/msg/wheel_control.hpp"
#include "amr_logger.hpp"

#include <Eigen/Dense>

#include "Serial.h"

using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace std::literals;

struct Pose
{
    double x;
    double y;
    double angle; // Orientation in radians
};

class PGV : public rclcpp::Node
{
public:
    PGV();
    ~PGV();

private:
    int serial_port_;
    unsigned char dir_straight_[2];
    unsigned char pos_req_[2];
    double agv_ang_des_;
    double agv_x_pos_des_;
    double agv_y_pos_des_;
    std::string selected_dir_;
    int agv_c_lane_cout_des_;
    int agv_no_color_lane_des_;
    int agv_no_pos_des_;
    int tag_detected_des_;
    double cal_err_x_;
    double cal_err_y_;
    double cal_err_ang_;

    char read_buf_[50];
    int pgv_count_;
    int pgv_report_;
    int pre_pgv_report_;

    double pgv_x_prev_;
    double pgv_y_prev_;
    double pgv_angle_prev_;
    // calib
    Pose pose_calib_1_;
    Pose pose_calib_2_;
    Pose pgv_pose_;
    int tag_id_;
    bool pose_flag_ = false;
    bool wheel_sub_flag_ = false;
    int64_t encoder_left_1_;
    int64_t encoder_right_1_;
    int64_t encoder_left_2_;
    int64_t encoder_right_2_;
    Eigen::Vector4d v_wheel_sub_ = Eigen::Vector4d::Zero();

public:
    void initVariables();
    void initSerial();
    unsigned long int string2decimal(std::string input);
    void pgvPublishingTimer();
    // calib
    double normalizeAngle(double angle);
    double normalizeAngleRad(double angle);
    double calculateDistance(const Pose &p1, const Pose &p2);
    void calculateWheelRadius(const Pose &p1, const Pose &p2,
                              int64_t enc_left, int64_t prev_enc_left,
                              int64_t enc_right, int64_t prev_enc_right,
                              int64_t enc_pulse, double gear_walk, double wheel_base);
    void wheelControlCallback(const tr_driver_msgs::msg::WheelControl::SharedPtr msg);
    void wheelRadiusCalib();

    void logWrite(LogLevel level, std::string log_string);

private:
    rclcpp::Publisher<tr_driver_msgs::msg::PgvData>::SharedPtr pgv_pub_;
    rclcpp::Subscription<tr_driver_msgs::msg::WheelControl>::SharedPtr wheel_control_sub_;
    rclcpp::TimerBase::SharedPtr pgv_pub_timer_;
};
#endif