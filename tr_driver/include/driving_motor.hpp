#include "rclcpp/rclcpp.hpp"
#include "can_open.hpp"
#include "std_msgs/msg/string.hpp"
#include "tr_driver_msgs/msg/motor_status.hpp"
#include "tr_driver_msgs/msg/motor_driver.hpp"
#include "tr_driver_msgs/msg/channel_status.hpp"
#include "tr_driver_msgs/msg/safety_status.hpp"

#include "tr_driver_msgs/srv/motor_run_position.hpp"
#include "tr_driver_msgs/srv/motor_run_velocity.hpp"
#include "tr_driver_msgs/srv/motor_stop_homing.hpp"
#include "tr_driver_msgs/srv/motor_send_var.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "tr_driver_msgs/msg/wheel_control.hpp"
#include "data_type/constants.h"
#include "data_type/alarm_data.hpp"
#include "helper/thread_safety_queue.hpp"
#include "helper/converter.hpp"
#include "helper/amr_logger.hpp"
#include "helper/file_io.hpp"
#include "nlohmann/json.hpp"
#include "motor_define.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <memory>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <signal.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <bitset>
#include <sys/time.h>
#include <fstream>
#include <mutex> // For std::mutex and std::lock_guard

using nlohmann::json;
using namespace std::placeholders;
using namespace std::chrono_literals;
using std::placeholders::_1;

const double PI = 3.14159266;

struct Error
{
    std::string error_name;
    int error_code;
};

class DrivingMotorsCanOpen : public rclcpp::Node
{
public:
    DrivingMotorsCanOpen();
    ~DrivingMotorsCanOpen();

    void declareParameters();
    void getParameters();
    void canOpenSysn();
    void init();
    void publishControlFeedbackCallback();
    void publishStatusFeedbackCallback();
    void publishJointStateCallback();
    void sendCommandCallback();

    void safetyStatusCallback(const tr_driver_msgs::msg::SafetyStatus::SharedPtr msg);
    void wheelMotorCallback(const tr_driver_msgs::msg::WheelControl::SharedPtr msg);
    
    // ===== 수정된 부분 1: 콜백 함수 선언 변경 =====
    // void jointDesiredCallback(const sensor_msgs::msg::JointState::SharedPtr msg); // 기존 선언 삭제
    void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg); // 새 선언 추가
    
    void logWrite(LogLevel level, std::string log_string);
    void findErrorMessage(int code);
    std::vector<bool> intToBinaryVector(uint16_t num, uint8_t length);
    std::string axis_state(uint16_t fault_code, bool axis_busy, bool homing_complete, bool homing, bool upper_limit, bool lower_limit, int32_t position);

    // service - server (기존과 동일)
    void motorRunPositionServerCallback(const std::shared_ptr<tr_driver_msgs::srv::MotorRunPosition::Request> request,
                                        std::shared_ptr<tr_driver_msgs::srv::MotorRunPosition::Response> response);
    void motorRunVelocityServerCallback(const std::shared_ptr<tr_driver_msgs::srv::MotorRunVelocity::Request> request,
                                        std::shared_ptr<tr_driver_msgs::srv::MotorRunVelocity::Response> response);
    void motorStopServerCallback(const std::shared_ptr<tr_driver_msgs::srv::MotorStopHoming::Request> request,
                                 std::shared_ptr<tr_driver_msgs::srv::MotorStopHoming::Response> response);
    void motorHomingServerCallback(const std::shared_ptr<tr_driver_msgs::srv::MotorStopHoming::Request> request,
                                   std::shared_ptr<tr_driver_msgs::srv::MotorStopHoming::Response> response);

    // Time Function (기존과 동일)
    unsigned int getTickCount();
    unsigned long elapseTime(unsigned int dw_start_tick_count);
    bool getTimeOut(unsigned int dw_start_tick_count, unsigned millisecound);

public: // var
    // 기존 public 멤버 변수들...
    std::vector<Error> errors = { /* ... 에러 목록 ... */ };
    // ... (나머지 public 변수들은 기존과 동일) ...

    std::string can_interface_;
    std::string dcf_path_;
    double motor_freq_ = 100.0;
    bool init_done_ = false;

    // ... (기타 public 멤버 변수들) ...
    
    std::thread canopen_thread_;
    std::thread SDO_thread_;

    std::mutex data_mutex;

    rclcpp::TimerBase::SharedPtr send_control_data_;
    rclcpp::TimerBase::SharedPtr send_status_data_;
    rclcpp::TimerBase::SharedPtr send_command_;
    rclcpp::TimerBase::SharedPtr joint_state_publisher_timer_;

    rclcpp::Subscription<tr_driver_msgs::msg::WheelControl>::SharedPtr wheel_motor_sub_;
    rclcpp::Subscription<tr_driver_msgs::msg::SafetyStatus>::SharedPtr safety_st_sub_;
    
    // ===== 수정된 부분 2: 구독자(subscriber) 변수 타입 변경 =====
    // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_desired_sub_; // 기존 변수 삭제
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_sub_; // 새 변수 추가

    rclcpp::Publisher<tr_driver_msgs::msg::WheelControl>::SharedPtr wheel_motor_pub_;
    rclcpp::Publisher<tr_driver_msgs::msg::WheelControl>::SharedPtr control_data_pub_;
    rclcpp::Publisher<tr_driver_msgs::msg::MotorStatus>::SharedPtr status_data_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    
    // 서비스 서버들 (기존과 동일)
    rclcpp::Service<tr_driver_msgs::srv::MotorRunPosition>::SharedPtr motor_run_position_server_;
    rclcpp::Service<tr_driver_msgs::srv::MotorRunVelocity>::SharedPtr motor_run_velocity_server_;
    rclcpp::Service<tr_driver_msgs::srv::MotorStopHoming>::SharedPtr motor_stop_server_;
    rclcpp::Service<tr_driver_msgs::srv::MotorStopHoming>::SharedPtr motor_homing_server_;

private:
    int num_driver = 1;
    
    // ===== 수정된 부분 3: 파라미터 저장을 위한 멤버 변수 추가 =====
    double gear_ratio_;
    double ticks_per_revolution_;
	double sample_time_;
    std::string joint_trajectory_topic_;

    std::vector<std::unique_ptr<VelocityControl>> drivers_unique_;
    std::vector<VelocityControl*> drivers;
    Tcon_Master *master;
};