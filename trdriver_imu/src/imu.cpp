#include "imu.hpp"

Imu::Imu() : Node("imu_node")
{
    RCLCPP_INFO(get_logger(), "imu_node init starts! ");

    // Create filters with custom frequency and damping using std::make_shared
    vel_filter_ = std::make_shared<LowPassFilter>(15, 0.707); // Filter for velocity
    acc_filter_ = std::make_shared<LowPassFilter>(15, 0.707); // Filter for acceleration

    vel_ekf_filter_ = std::make_shared<ExtendedKalmanFilter>(0.008, 0.04); // Filter for velocity
    acc_ekf_filter_ = std::make_shared<ExtendedKalmanFilter>(0.008, 0.04); // Filter for acceleration
    yaw_ekf_filter_ = std::make_shared<ExtendedKalmanFilter>(0.004, 0.03); // Filter for acceleration

    declareParameters();
    getParameters();

    initSerial();

    /*Pub*/
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::QoS(rclcpp::KeepLast(10)));
    srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
        "imu/reset", std::bind(&Imu::reset, this, std::placeholders::_1, std::placeholders::_2));

    /*Timer*/
    imu_pub_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / imu_freq_), std::bind(&Imu::imuPublishingTimer, this));
    read_thread_ = std::make_shared<std::thread>(&Imu::read, this);

    RCLCPP_INFO(get_logger(), "imu_node init finished! ");
}

Imu::~Imu()
{
    serial_->Close();
}
void Imu::declareParameters()
{
    RCLCPP_INFO(get_logger(), "imu_node declareParameters starts! ");
    this->declare_parameter("imu_topic", "raw_imu");
    this->declare_parameter("calibrate_imu", true);
    this->declare_parameter("imu_freq", 100.0);
    RCLCPP_INFO(get_logger(), "imu_node declareParameters finished! ");
}
void Imu::getParameters()
{
    RCLCPP_INFO(get_logger(), "odom_node getParameters starts! ");

    this->get_parameter<std::string>("imu_topic", imu_topic_);
    this->get_parameter<bool>("calibrate_imu", calibrate_imu_);
    this->get_parameter<double>("imu_freq", imu_freq_);

    RCLCPP_INFO(get_logger(), "Imu Topic: %s", imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Caibrate IMU: %s", calibrate_imu_ ? "True" : "False");
    RCLCPP_INFO(get_logger(), "Imu Freq: %f", imu_freq_);

    RCLCPP_INFO(get_logger(), "odom_node getParameters finished! ");
}
void Imu::initSerial()
{
    // Initial setting for serial communication
    serial_ = std::make_shared<LibSerial::SerialPort>();

    try
    {
        serial_->Open(port_);
    }
    catch (LibSerial::OpenFailed &e)
    {
        RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "LibSerial::OpenFailed: " << e.what());
        return;
    }

    serial_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_->SetParity(LibSerial::Parity::PARITY_NONE);
    serial_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_->FlushIOBuffers();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Check the serial port
    if (serial_->IsOpen())
    {
        RCLCPP_INFO(this->get_logger(), "MW AHRS driver connected to %s at %i baud", port_.c_str(), 115200);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "MW AHRS driver failed to connect to %s", port_.c_str());
        return;
    }

    start();
}
void Imu::read()
{
    while (rclcpp::ok())
    {
        if (serial_->IsDataAvailable())
        {
            std::string msg;
            serial_->ReadLine(msg, eol, timeout);
            // RCLCPP_INFO_STREAM(this->get_logger(), "Message : " << msg);

            int cnt = 0;
            std::string data[1000];
            char *buff = new char[1000];
            strcpy(buff, msg.c_str());
            char *tok = strtok(buff, " ");

            // Variables to hold parsed values
            ImuAhrs temp_ahrs_obj;

            while (tok != nullptr)
            {
                data[cnt++] = std::string(tok);
                tok = strtok(nullptr, " ");
            }
            // RCLCPP_INFO_STREAM(this->get_logger(), "cnt %d: " << cnt);
            if (cnt == 12) // Acceleration, angular velocity, angle, magnetic data
            {
                ahrs_obj_.a_y = stod(data[0]) * 9.80665;
                ahrs_obj_.a_x = stod(data[1]) * 9.80665;
                ahrs_obj_.a_z = stod(data[2]) * 9.80665;

                ahrs_obj_.g_x = stod(data[3]) * M_PI / 180.0;
                ahrs_obj_.g_y = stod(data[4]) * M_PI / 180.0;
                ahrs_obj_.g_z = stod(data[5]) * M_PI / 180.0;

                temp_ahrs_obj.e_roll = stod(data[6]) * M_PI / 180.0;
                temp_ahrs_obj.e_pitch = stod(data[7]) * M_PI / 180.0;
                temp_ahrs_obj.e_yaw = stod(data[8]) * M_PI / 180.0;

                ahrs_obj_.m_x = stod(data[9]) * 0.000001;
                ahrs_obj_.m_y = stod(data[10]) * 0.000001;
                ahrs_obj_.m_z = stod(data[11]) * 0.000001;

                if (first_flag_ && (temp_ahrs_obj.e_yaw != 0.0))
                {
                    first_flag_ = false;
                    yaw_offset_ = temp_ahrs_obj.e_yaw;
                }

                if (temp_ahrs_obj.e_yaw != 0.0)
                {
                    temp_ahrs_obj.e_yaw -= yaw_offset_;
                }
                // Convert roll, pitch, yaw to radians and store
                ahrs_obj_.e_roll = temp_ahrs_obj.e_roll;
                ahrs_obj_.e_pitch = temp_ahrs_obj.e_pitch;
                ahrs_obj_.e_yaw = temp_ahrs_obj.e_yaw;
                ahrs_obj_.e_yaw = normalizeAngle(ahrs_obj_.e_yaw);
                // RCLCPP_INFO(this->get_logger(), "Processed IMU Data (rpy in radians): %f, %f, %f",
                //             ahrs_obj_.e_roll, ahrs_obj_.e_pitch, ahrs_obj_.e_yaw * 180.0 / M_PI);
            }
        }
    }
}
void Imu::start()
{
    std::string start = "ss=15\n"; // 15 Acceleration, angular velocity, angle, magnetic data
    serial_->Write(start);

    int int_sampling_time = static_cast<int>(1000.0 / imu_freq_);         // Convert to integer ms
    std::string string_sampling_time = std::to_string(int_sampling_time); // Convert to string
    std::string frequency = "sp=" + string_sampling_time + "\n";          // Format the string

    // std::string frequency = "sp=10\n"; // 10 ms
    serial_->Write(frequency);
}

void Imu::reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
    std::string msg = "rst\n";
    serial_->Write(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    start();
    resp->success = true;
}

void Imu::imuPublishingTimer()
{
    auto raw_imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

    if (calibrate_imu_ && calibration_start_)
    {
        int i = init_calibration_cnt_;

        if (init_calibration_cnt_ == 0)
        {
            RCLCPP_INFO(get_logger(), "Running IMU calibration...");
        }

        acc_x_bias = ((acc_x_bias * i) + ahrs_obj_.a_x) / (i + 1);
        acc_y_bias = ((acc_y_bias * i) + ahrs_obj_.a_y) / (i + 1);
        acc_z_bias = ((acc_z_bias * i) + ahrs_obj_.a_z - 9.81) / (i + 1);
        vel_theta_bias = ((vel_theta_bias * i) + ahrs_obj_.g_z) / (i + 1);

        init_calibration_cnt_++;

        if (init_calibration_cnt_ >= SAMPLE_IMU_TIMES)
        {
            calibration_start_ = false;
            init_calibration_cnt_ = 0;
            RCLCPP_INFO(get_logger(), "IMU calibration done. Bias: %.2f %.2f %.2f %.2f",
                        acc_x_bias, acc_y_bias, acc_z_bias, vel_theta_bias);
        }
    }
    ImuAhrs ahrs_obj_calib;
    ahrs_obj_calib.a_x = ahrs_obj_.a_x - acc_x_bias;
    ahrs_obj_calib.g_z = ahrs_obj_.g_z - vel_theta_bias;
    double sampling_time = 1.0 / imu_freq_;
    // filter
    // double ax_filter = acc_filter_->applyLPF(ahrs_obj_calib.a_x, sampling_time);
    // double gz_filter = vel_filter_->applyLPF(ahrs_obj_calib.g_z, sampling_time);

    // acc_ekf_filter_->predict(sampling_time);
    // acc_ekf_filter_->update(ahrs_obj_calib.a_x);
    // double ax_ekf_filter = acc_ekf_filter_->getFiltered();

    // vel_ekf_filter_->predict(sampling_time);
    // vel_ekf_filter_->update(ahrs_obj_.g_z);
    // double gz_ekf_filter = vel_ekf_filter_->getFiltered();
    // yaw filter
    yaw_ekf_filter_->predict(sampling_time);
    yaw_ekf_filter_->update(ahrs_obj_.e_yaw);
    double imu_yaw = yaw_ekf_filter_->getFiltered();

    raw_imu_msg->header.frame_id = "imu_link";
    raw_imu_msg->header.stamp = this->now();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, ahrs_obj_.e_yaw);

    raw_imu_msg->orientation.x = q.x();
    raw_imu_msg->orientation.y = q.y();
    raw_imu_msg->orientation.z = q.z();
    raw_imu_msg->orientation.w = q.w();

    raw_imu_msg->orientation_covariance[0] = -1; // we don't have estimation for orientation
    raw_imu_msg->linear_acceleration.x = ahrs_obj_.a_x - acc_x_bias;
    raw_imu_msg->linear_acceleration.y = ahrs_obj_.a_y - acc_y_bias;
    raw_imu_msg->linear_acceleration.z = ahrs_obj_.a_z - acc_z_bias;
    raw_imu_msg->linear_acceleration_covariance.fill(0.0);
    raw_imu_msg->linear_acceleration_covariance[0] = 0.01;
    raw_imu_msg->linear_acceleration_covariance[4] = 0.01;
    raw_imu_msg->linear_acceleration_covariance[8] = 0.01;
    raw_imu_msg->angular_velocity.x = ahrs_obj_.g_x;
    raw_imu_msg->angular_velocity.y = ahrs_obj_.g_y;
    raw_imu_msg->angular_velocity.z = ahrs_obj_.g_z;
    raw_imu_msg->angular_velocity_covariance.fill(0.0);
    raw_imu_msg->angular_velocity_covariance[0] = 0.01;
    raw_imu_msg->angular_velocity_covariance[4] = 0.01;
    raw_imu_msg->angular_velocity_covariance[8] = 0.01;

    imu_pub_->publish(std::move(raw_imu_msg));
    // RCLCPP_INFO(this->get_logger(), "ahrs_obj_.e_yaw:  %f %f", ahrs_obj_.e_yaw, imu_yaw);
}
double Imu::normalizeAngle(double angle)
{
    // Normalize angle to be between -pi and pi
    if (angle > M_PI)
        angle -= 2 * M_PI;
    else if (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}
