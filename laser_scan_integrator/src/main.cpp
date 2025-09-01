//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//   Modified for robust use_sim_time handling
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/create_timer_ros.h>

#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <limits> // Required for std::numeric_limits

struct LaserPoint
{
    float direction_;
    float distance_;
};

struct LaserPointLess
{
  bool operator()(const LaserPoint& a, const LaserPoint& b) const noexcept {
    return a.direction_ < b.direction_;
  }
};

class scanMerger : public rclcpp::Node
{
    public:
    // 생성자에 rclcpp::NodeOptions 추가
    scanMerger(const rclcpp::NodeOptions & options)
    : Node("ros2_laser_scan_merger", options)
    {
        tolerance_ = this->declare_parameter("transform_tolerance", 0.01);

        refresh_params();
        
        
        auto default_qos = rclcpp::SensorDataQoS();
        sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic1_, default_qos, std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));
        sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic2_, default_qos, std::bind(&scanMerger::scan_callback2 , this, std::placeholders::_1));
        
        laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(integratedTopic_, rclcpp::SystemDefaultsQoS());
        
        tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
       
    }

    private:
    void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        laser1_ = _msg;
        if (laser2_)
        {
          update_point_cloud_rgb();
        }
    }

    void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        laser2_ = _msg;
    }
    
    void update_point_cloud_rgb(){
        refresh_params();

        // lookupTransform 호출 방식 변경: rclcpp::Time(0) 대신 메시지의 타임스탬프를 사용
        try {
            trans1_ = tf2_->lookupTransform(integratedFrameId_, laser1_->header.frame_id, laser1_->header.stamp, tf2::durationFromSec(tolerance_));
            trans2_ = tf2_->lookupTransform(integratedFrameId_, laser2_->header.frame_id, laser2_->header.stamp, tf2::durationFromSec(tolerance_));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s or %s to %s: %s",
                laser1_->header.frame_id.c_str(),
                laser2_->header.frame_id.c_str(),
                integratedFrameId_.c_str(),
                ex.what());
            return;
        }

        double sensor1_r, sensor1_p, sensor1_y, sensor2_r, sensor2_p, sensor2_y;
        geometry_quat_to_rpy(&sensor1_r, &sensor1_p, &sensor1_y, trans1_.transform.rotation);
        geometry_quat_to_rpy(&sensor2_r, &sensor2_p, &sensor2_y, trans2_.transform.rotation);
        sensor1_y += laser1Yaw_;
        sensor2_y += laser2Yaw_;
        std::vector<std::array<float,2>> scan_data;
        int count = 0;
        
        if(show1_){
            for (size_t i = 0; i < laser1_->ranges.size(); ++i){
                float current_angle = laser1_->angle_min + i * laser1_->angle_increment;
                std::array<float, 2> pt;
                float laser_angle;

                if (fabs(sensor1_r) < M_PI/2) {
                    laser_angle = current_angle;
                } else {
                    laser_angle = -current_angle;
                }

                float temp_x = laser1_->ranges[i] * std::cos(laser_angle);
                float temp_y = laser1_->ranges[i] * std::sin(laser_angle);
                pt[0] = temp_x * std::cos(sensor1_y) - temp_y * std::sin(sensor1_y);
                pt[0] += trans1_.transform.translation.x + laser1XOff_;
                pt[1] = temp_x * std::sin(sensor1_y) + temp_y * std::cos(sensor1_y);
                pt[1] += trans1_.transform.translation.y + laser1YOff_;

                if (pt[0] < robotFrontEnd_ && pt[0] > -robotRearEnd_ && pt[1] < robotLeftEnd_ && pt[1] > -robotRightEnd_) {
                    continue;
                }
                
                std::array<float,2> res_;
                res_[1] = GET_R(pt[0], pt[1]); // distance
                res_[0] = GET_THETA(pt[0], pt[1]); // angle
                scan_data.push_back(res_);
            }
        }

        if(show2_){
            for (size_t i = 0; i < laser2_->ranges.size(); ++i){
                float current_angle = laser2_->angle_min + i * laser2_->angle_increment;
                std::array<float,2> pt;
                float laser_angle;

                if (fabs(sensor2_r) < M_PI/2) {
                    laser_angle = current_angle;
                } else {
                    laser_angle = -current_angle;
                }

                float temp_x = laser2_->ranges[i] * std::cos(laser_angle);
                float temp_y = laser2_->ranges[i] * std::sin(laser_angle);
                pt[0] = temp_x * std::cos(sensor2_y) - temp_y * std::sin(sensor2_y);
                pt[0] += trans2_.transform.translation.x + laser2XOff_;
                pt[1] = temp_x * std::sin(sensor2_y) + temp_y * std::cos(sensor2_y);
                pt[1] += trans2_.transform.translation.y + laser2YOff_;
                
                if (pt[0] < robotFrontEnd_ && pt[0] > -robotRearEnd_ && pt[1] < robotLeftEnd_ && pt[1] > -robotRightEnd_) {
                    continue;
                }

                std::array<float,2> res_;
                res_[1] = GET_R(pt[0], pt[1]); // distance
                res_[0] = GET_THETA(pt[0], pt[1]); // angle
                scan_data.push_back(res_);
            }
        }

        // 통합 스캔 메시지 준비
        auto integrated_msg_ = std::make_unique<sensor_msgs::msg::LaserScan>();
        integrated_msg_->header.frame_id = integratedFrameId_;
        integrated_msg_->header.stamp    = this->now(); // 현재 시간(sim time or system time) 사용

        // 두 라이다가 동일한 해상도를 가진다고 가정
        const float inc = laser1_->angle_increment;
        const float snapped_min = -M_PI;
        int n_bins = static_cast<int>(std::floor((2*M_PI) / inc)) + 1;

        if (n_bins <= 0 || scan_data.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No valid scan data to publish.");
            return;
        }

        // 모든 bin을 inf로 초기화
        std::vector<float> ranges(n_bins, std::numeric_limits<float>::infinity());
        
        // 각 포인트를 가장 가까운 bin으로 매핑하여 최소 거리 기록
        for (const auto& a : scan_data) {
            const float theta = a[0];
            const float r     = a[1];

            if (!std::isfinite(theta) || !std::isfinite(r)) continue;
            if (r < laser1_->range_min || r > laser1_->range_max) continue;

            int idx = static_cast<int>(std::round((theta - snapped_min) / inc));
            if (0 <= idx && idx < n_bins) {
                if (r < ranges[idx]) ranges[idx] = r;  // 같은 bin에는 더 가까운 값 유지
            }
        }

        integrated_msg_->angle_min       = snapped_min;
        integrated_msg_->angle_max       = snapped_min + (n_bins - 1) * inc;
        integrated_msg_->angle_increment = inc;
        integrated_msg_->time_increment  = laser1_->time_increment;
        integrated_msg_->scan_time       = laser1_->scan_time;
        integrated_msg_->range_min       = laser1_->range_min;
        integrated_msg_->range_max       = laser1_->range_max;
        integrated_msg_->ranges          = std::move(ranges);
        
        laser_scan_pub_->publish(std::move(integrated_msg_));
    }

    float GET_R(float x, float y){
        return sqrt(x*x + y*y);
    }

    float GET_THETA(float x, float y){
        return atan2(y, x);
    }

    float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle){
        return (magnitude_1 + (current_angle - angle_1) * ((magnitude_2 - magnitude_1)/(angle_2 - angle_1)));
    }

    void geometry_quat_to_rpy(double* roll, double* pitch, double* yaw, geometry_msgs::msg::Quaternion geometry_quat){
        tf2::Quaternion quat;
        tf2::convert(geometry_quat, quat);
        tf2::Matrix3x3(quat).getRPY(*roll, *pitch, *yaw);
    }

    void refresh_params(){
        this->get_parameter("integratedTopic", integratedTopic_);
        this->get_parameter("integratedFrameId", integratedFrameId_);
        this->get_parameter("scanTopic1", topic1_);
        this->get_parameter("laser1XOff", laser1XOff_);
        this->get_parameter("laser1YOff", laser1YOff_);
        this->get_parameter("laser1Yaw", laser1Yaw_);
        this->get_parameter("show1", show1_);
        this->get_parameter("scanTopic2", topic2_);
        this->get_parameter("laser2XOff", laser2XOff_);
        this->get_parameter("laser2YOff", laser2YOff_);
        this->get_parameter("laser2Yaw", laser2Yaw_);
        this->get_parameter("show2", show2_);
        this->get_parameter("robotFrontEnd", robotFrontEnd_);
        this->get_parameter("robotRearEnd", robotRearEnd_);
        this->get_parameter("robotRightEnd", robotRightEnd_);
        this->get_parameter("robotLeftEnd", robotLeftEnd_);
    }
    
    std::string topic1_, topic2_, integratedTopic_, integratedFrameId_;
    bool show1_, show2_;
    float laser1XOff_, laser1YOff_, laser1Yaw_;
    float laser2XOff_, laser2YOff_, laser2Yaw_;
    float robotFrontEnd_, robotRearEnd_, robotRightEnd_, robotLeftEnd_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf2_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    sensor_msgs::msg::LaserScan::SharedPtr laser1_;
    sensor_msgs::msg::LaserScan::SharedPtr laser2_;
    geometry_msgs::msg::TransformStamped trans1_;
    geometry_msgs::msg::TransformStamped trans2_;
    
    double tolerance_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    
    // NodeOptions를 사용하여 use_sim_time 파라미터를 자동으로 처리
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    
    auto node = std::make_shared<scanMerger>(options);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}