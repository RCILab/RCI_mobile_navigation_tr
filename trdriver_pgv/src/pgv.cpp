#include "pgv.hpp"

PGV::PGV() : Node("pgv_node")
{
    RCLCPP_INFO(get_logger(), "pgv_node init starts! ");

    initVariables();
    initSerial();

    /*Pub*/
    pgv_pub_ = this->create_publisher<tr_driver_msgs::msg::PgvData>("pgv", rclcpp::QoS(rclcpp::KeepLast(10)));
    // ros Sub topic init
    wheel_control_sub_ = this->create_subscription<tr_driver_msgs::msg::WheelControl>("wheel_motor_state", 10, std::bind(&PGV::wheelControlCallback, this, _1));
    // /*Timer*/
    pgv_pub_timer_ = this->create_wall_timer(20ms, std::bind(&PGV::pgvPublishingTimer, this));

    RCLCPP_INFO(get_logger(), "pgv_node init finished! ");
}
PGV::~PGV()
{
    if (serial_port_ != -1) {
        close_serial(serial_port_);
    }
}
void PGV::initVariables()
{
    dir_straight_[0] = 0xEC;
    dir_straight_[1] = 0x13;

    pgv_count_ = 0;
    pgv_report_ = 0;
    pre_pgv_report_ = pgv_report_;

    pos_req_[0] = 0xc8;
    pos_req_[1] = 0x37;

    agv_ang_des_ = 0.0;
    agv_x_pos_des_ = 0.0;
    agv_no_color_lane_des_ = 1;
    agv_no_pos_des_ = 1;
    tag_detected_des_ = 0;
    cal_err_x_ = 0;
    cal_err_y_ = 0;
    cal_err_ang_ = 0;
    serial_port_ = -1;
}
void PGV::initSerial()
{
    rclcpp::WallRate loop_rate(100ms);

    while (serial_port_ == -1 && rclcpp::ok())
    {
        serial_port_ = open_serial((char *)"/dev/ttyUSB0", 115200, 0, 0);

        if (serial_port_ != -1)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully opened serial port /dev/ttyUSB0.");
            pgv_report_ = 1;
            rclcpp::Rate loop_rate(10);
            selected_dir_ = "No lane is selected";
            write(serial_port_, dir_straight_, 2);
            loop_rate.sleep();
            read(serial_port_, read_buf_, sizeof(char) * 3);
            write(serial_port_, pos_req_, 2);
        }
        else
        {
            // [수정] 시리얼 포트 연결 실패 시 경고 메시지 출력
            RCLCPP_WARN(this->get_logger(), "Failed to open serial port /dev/ttyUSB0. Retrying...");
            loop_rate.sleep();
        }
    }
}
unsigned long int PGV::string2decimal(std::string input)
{
    char *pEnd;
    unsigned long int out_dec = strtoull(input.c_str(), &pEnd, 2);
    return out_dec;
}

void PGV::pgvPublishingTimer()
{
    if (serial_port_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not available.");
        return;
    }

    memset(&read_buf_, '\0', sizeof(read_buf_));
    int byte_count = read(serial_port_, read_buf_, sizeof(char) * 50);

    if (byte_count < 0)
    {
        pgv_count_++;

        if (pgv_count_ >= 100)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read from serial port. Re-initializing...");
            pgv_report_ = 0;
            close_serial(serial_port_);
            serial_port_ = -1;
            initSerial();
        }
        return;
    }

    pgv_count_ = 0;

    for (; byte_count < 21 && rclcpp::ok();)
    {
        char temp_buf[21];
        int temp_cnt = read(serial_port_, temp_buf, sizeof(char) * 21);
        if (temp_cnt > 0) {
            memcpy(read_buf_ + byte_count, temp_buf, temp_cnt);
            byte_count += temp_cnt;
        }
    }

    // 데이터 파싱 부분 (기존과 동일)
    std::bitset<7> lane_detect_byte1(read_buf_[0]);
    std::bitset<7> lane_detect_byte0(read_buf_[1]);
    std::string agv_lane_detect_str = lane_detect_byte1.to_string() + lane_detect_byte0.to_string();

    std::string agv_c_lane_count_str = agv_lane_detect_str.substr(8, 2);
    std::string agv_c_lane_detect_str = agv_lane_detect_str.substr(11, 1);
    std::string agv_no_pos_str = agv_lane_detect_str.substr(5, 1);
    std::string tag_detected = agv_lane_detect_str.substr(7, 1);

    int agv_c_lane_count_des = string2decimal(agv_c_lane_count_str);
    int agv_no_color_lane_des_ = string2decimal(agv_c_lane_detect_str);
    int agv_no_pos_des_ = string2decimal(agv_no_pos_str);
    tag_detected_des_ = string2decimal(tag_detected);

    std::bitset<7> ang_1(read_buf_[10]);
    std::bitset<7> ang_0(read_buf_[11]);
    std::string agv_ang_str = ang_1.to_string() + ang_0.to_string();
    agv_ang_des_ = string2decimal(agv_ang_str) * (-0.1);
    agv_ang_des_ = normalizeAngle(agv_ang_des_);

    std::bitset<3> x_pos_3(read_buf_[2]);
    std::bitset<7> x_pos_2(read_buf_[3]);
    std::bitset<7> x_pos_1(read_buf_[4]);
    std::bitset<7> x_pos_0(read_buf_[5]);
    std::string agv_x_pos_str = x_pos_3.to_string() + x_pos_2.to_string() + x_pos_1.to_string() + x_pos_0.to_string();
    agv_x_pos_des_ = string2decimal(agv_x_pos_str);
    if (tag_detected_des_ != 0)
    {
        if (agv_x_pos_des_ > 2000.0)
            agv_x_pos_des_ = agv_x_pos_des_ - pow(2, 24) - 1;
    }

    std::bitset<7> y_pos_1(read_buf_[6]);
    std::bitset<7> y_pos_0(read_buf_[7]);
    std::string agv_y_pos_str = y_pos_1.to_string() + y_pos_0.to_string();
    agv_y_pos_des_ = string2decimal(agv_y_pos_str);
    if (agv_y_pos_des_ > 2000.0)
        agv_y_pos_des_ = agv_y_pos_des_ - 16383.0;

    if (agv_no_pos_des_)
        agv_y_pos_des_ *= -1;

    std::bitset<7> tag_3(read_buf_[14]);
    std::bitset<7> tag_2(read_buf_[15]);
    std::bitset<7> tag_1(read_buf_[16]);
    std::bitset<7> tag_0(read_buf_[17]);
    std::string tag_id_str = tag_3.to_string() + tag_2.to_string() + tag_1.to_string() + tag_0.to_string();
    int tag_id = string2decimal(tag_id_str);

    // 메시지 채우기
    tr_driver_msgs::msg::PgvData pgv_msg;

    read_buf_[0] &= 0x30;
    if (read_buf_[0] == 0)
        pgv_msg.pgv_id = 0;
    else
        pgv_msg.pgv_id = 1;
    pgv_msg.angle = agv_ang_des_ - cal_err_ang_;
    pgv_msg.x_pos = (agv_x_pos_des_ - cal_err_x_) / 10.0;
    pgv_msg.y_pos = (agv_y_pos_des_ - cal_err_y_) / 10.0;
    pgv_msg.direction = selected_dir_;
    pgv_msg.color_lane_count = agv_c_lane_count_des;
    pgv_msg.no_color_lane = agv_no_color_lane_des_;
    pgv_msg.no_pos = agv_no_pos_des_;
    pgv_msg.tag_detected = tag_detected_des_;
    pgv_msg.tag_id = tag_id;
    pgv_msg.header.stamp = this->now();

    // [수정] 퍼블리시 직전에 항상 로그를 출력하여 터미널에서 확인
    RCLCPP_INFO(this->get_logger(), "Publishing PGV Data -> Tag ID: [%d], Pos (x, y, ang): [%.2f, %.2f, %.2f]",
                pgv_msg.tag_id, pgv_msg.x_pos, pgv_msg.y_pos, pgv_msg.angle);

    pgv_pub_->publish(pgv_msg);

    write(serial_port_, pos_req_, 2);
    
    // 이 아래의 기존 로그는 큰 움직임이 있을 때만 기록하므로 그대로 둡니다.
    if (pgv_msg.x_pos != 0.0 || pgv_msg.y_pos != 0.0 || pgv_msg.angle != 0.0)
    {
        if (abs(pgv_msg.x_pos - pgv_x_prev_) > 0.4 || abs(pgv_msg.y_pos - pgv_y_prev_) > 0.4 || (pgv_msg.angle - pgv_angle_prev_) > 0.2)
        {
            std::string pgv_log = std::to_string(pgv_msg.tag_id) + " " + std::to_string(pgv_msg.x_pos) + " " + std::to_string(pgv_msg.y_pos) + " " + std::to_string(pgv_msg.angle);
            logWrite(LOG_INFO, pgv_log);
        }
    }
    pgv_x_prev_ = pgv_msg.x_pos;
    pgv_y_prev_ = pgv_msg.y_pos;
    pgv_angle_prev_ = pgv_msg.angle;

    pgv_pose_.x = pgv_msg.x_pos / 1000.0; // unit m
    pgv_pose_.y = pgv_msg.y_pos / 1000.0;
    pgv_pose_.angle = pgv_msg.angle * M_PI / 180.0;
    tag_id_ = tag_id;

    wheelRadiusCalib();
}
void PGV::wheelControlCallback(const tr_driver_msgs::msg::WheelControl::SharedPtr msg)
{
    // WheelControl.msg에 상수를 정의했으므로 그대로 사용합니다.
    v_wheel_sub_[0] = msg->data[tr_driver_msgs::msg::WheelControl::LEFT_WALK].position;
    v_wheel_sub_[1] = msg->data[tr_driver_msgs::msg::WheelControl::RIGHT_WALK].position;

    wheel_sub_flag_ = true;
}
void PGV::wheelRadiusCalib()
{
    if (!pose_flag_ && tag_id_ == 57 && wheel_sub_flag_)
    {
        pose_calib_1_.x = pgv_pose_.x;
        pose_calib_1_.y = pgv_pose_.y;
        pose_calib_1_.angle = pgv_pose_.angle;
        encoder_left_1_ = (int64_t)v_wheel_sub_[0];
        encoder_right_1_ = (int64_t)v_wheel_sub_[1];
        pose_flag_ = true;
    }
    else if ((tag_id_ == 2 || (tag_id_ == 0 && (pgv_pose_.x != 0.0 || pgv_pose_.y != 0.0)) || tag_id_ == 58) && wheel_sub_flag_)
    {
        pose_calib_2_.x = pgv_pose_.x + 2.7;
        if (tag_id_ == 2)
        {
            pose_calib_2_.y = pgv_pose_.y + 0.1;
        }
        else if (tag_id_ == 0)
        {
            pose_calib_2_.y = pgv_pose_.y - 0.1;
        }
        else
        {
            pose_calib_2_.y = pgv_pose_.y;
        }

        pose_calib_2_.angle = pgv_pose_.angle;
        encoder_left_2_ = (int64_t)v_wheel_sub_[0];
        encoder_right_2_ = (int64_t)v_wheel_sub_[1];
        int64_t enc_pulse = 65536;
        double wheel_base = 0.408;
        double gear_walk = 20.0;
        calculateWheelRadius(pose_calib_1_, pose_calib_2_,
                             encoder_left_2_, encoder_left_1_,
                             encoder_right_2_, encoder_right_1_,
                             enc_pulse, gear_walk, wheel_base);
        pose_flag_ = false;
    }
    wheel_sub_flag_ = false;
}
double PGV::normalizeAngle(double angle)
{
    if (angle > 180.0)
        angle -= 360.0;
    else if (angle < -180.0)
        angle += 360.0;
    return angle;
}
double PGV::normalizeAngleRad(double angle)
{
    if (angle > M_PI)
        angle -= 2 * M_PI;
    else if (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}
double PGV::calculateDistance(const Pose &p1, const Pose &p2)
{
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}
void PGV::calculateWheelRadius(const Pose &p1, const Pose &p2,
                                 int64_t enc_left, int64_t prev_enc_left,
                                 int64_t enc_right, int64_t prev_enc_right,
                                 int64_t enc_pulse, double gear_walk, double wheel_base)
{
    double delta_s = calculateDistance(p1, p2);
    double delta_theta = normalizeAngleRad(p2.angle - p1.angle);
    double delta_s_left = delta_s - (wheel_base / 2.0) * delta_theta;
    double delta_s_right = delta_s + (wheel_base / 2.0) * delta_theta;
    double wheel_rotation_left = (enc_left - prev_enc_left) / (double)enc_pulse / gear_walk;
    double wheel_rotation_right = (enc_right - prev_enc_right) / (double)enc_pulse / gear_walk;
    double radius_left = delta_s_left / (wheel_rotation_left * 2 * M_PI);
    double radius_right = delta_s_right / (wheel_rotation_right * 2 * M_PI);
    // 이 변수들은 현재 사용되지 않으므로, 값을 출력하거나 나중에 사용할 수 있습니다.
    (void)radius_left;
    (void)radius_right;
}
void PGV::logWrite(LogLevel level, std::string log_string)
{
    auto &logger = AmrLogger::getInstance();
    logger.logWrite(LOG_PGV, level, log_string);
}
