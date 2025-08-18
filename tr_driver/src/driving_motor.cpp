#include "driving_motor.hpp"
#include <lely/util/diag.h>
#include <cmath> // M_PI와 같은 수학 상수 사용을 위해 추가
#include <trajectory_msgs/msg/joint_trajectory.hpp> // JointTrajectory 메시지 사용을 위해 추가

// Lely 라이브러리 로그 핸들러 (기존과 동일)
void lelyLogHandler(void * /*handle*/, enum diag_severity severity, int /*errc*/,
                    const char *format, va_list ap)
{
    if (severity < DIAG_DEBUG)
        return;

    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, ap);

    std::cerr << "[LELY][" << severity << "] " << buffer << std::endl;
}

// 생성자: 구독자 및 발행자 설정
DrivingMotorsCanOpen::DrivingMotorsCanOpen() : Node("driving_motor")
{
    logWrite(LOG_INFO, "driving_motor node is started");
    declareParameters();
    getParameters();

    canopen_thread_ = std::thread(&DrivingMotorsCanOpen::canOpenSysn, this);
    logWrite(LOG_INFO, "creating a thread is done.");

    sample_time_ = 1.0 / motor_freq_;

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("drives/joint_states", 10);
    joint_state_publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&DrivingMotorsCanOpen::publishJointStateCallback, this));

    // JointTrajectory 메시지를 구독하도록 수정
    joint_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        joint_trajectory_topic_, // 파라미터로 받은 토픽 이름 사용
        10,
        std::bind(&DrivingMotorsCanOpen::jointTrajectoryCallback, this, _1));
}

// 소멸자 (기존과 동일)
DrivingMotorsCanOpen::~DrivingMotorsCanOpen()
{
    if (canopen_thread_.joinable())
    {
        canopen_thread_.join();
    }
}

// 파라미터 선언: 모터 사양을 기본값으로 반영
void DrivingMotorsCanOpen::declareParameters()
{
    RCLCPP_INFO(get_logger(), "driving_motor_node declareParameters starts! ");
    this->declare_parameter("motor_freq", 100.0);
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("dcf_path", "");
    this->declare_parameter("num_driver", 1);
    // 제공된 모터 사양을 기본값으로 설정
    this->declare_parameter("gear_ratio", 20.0);
    this->declare_parameter("ticks_per_revolution", 65536.0);
    this->declare_parameter("joint_trajectory_topic", "drives/joint_trajectory");
    RCLCPP_INFO(get_logger(), "driving_motor_node declareParameters finished! ");
}

// 파라미터 읽기: 단위 변환에 필요한 파라미터 추가
void DrivingMotorsCanOpen::getParameters()
{
    RCLCPP_INFO(get_logger(), "driving_motor_node getParameters starts! ");
    this->get_parameter<double>("motor_freq", motor_freq_);
    this->get_parameter<std::string>("can_interface", can_interface_);
    this->get_parameter<std::string>("dcf_path", dcf_path_);
    this->get_parameter<int>("num_driver", num_driver);
    // 단위 변환에 필요한 파라미터 읽기
    this->get_parameter<double>("gear_ratio", gear_ratio_);
    this->get_parameter<double>("ticks_per_revolution", ticks_per_revolution_);
    this->get_parameter<std::string>("joint_trajectory_topic", joint_trajectory_topic_);

    RCLCPP_INFO(get_logger(), "Motor Freq: %f", motor_freq_);
    RCLCPP_INFO(get_logger(), "CAN Interface: %s", can_interface_.c_str());
    RCLCPP_INFO(get_logger(), "DCF Path: %s", dcf_path_.c_str());
    RCLCPP_INFO(get_logger(), "Num Driver: %d", num_driver);
    RCLCPP_INFO(get_logger(), "Gear Ratio: %f", gear_ratio_);
    RCLCPP_INFO(get_logger(), "Ticks Per Revolution: %f", ticks_per_revolution_);
    RCLCPP_INFO(get_logger(), "Subscribing to: %s", joint_trajectory_topic_.c_str());
    RCLCPP_INFO(get_logger(), "driving_motor_node getParameters finished! ");
}

// JointTrajectory 콜백 함수: 수신된 속도 명령 처리
void DrivingMotorsCanOpen::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (!init_done_ || msg->points.empty())
    {
        return;
    }

    const auto &point = msg->points[0];

    for (size_t i = 0; i < msg->joint_names.size(); ++i)
    {
        const std::string &joint_name = msg->joint_names[i];

        if (i >= point.velocities.size())
        {
            continue;
        }

        const double velocity_rad_s = point.velocities[i]; // rad/s 단위의 목표 속도

        // 1. 단위를 rad/s에서 RPM으로 변환합니다.
        //    (rad/s -> 초당 회전수 -> 분당 회전수) * 기어비
        const double TWO_PI = 2.0 * M_PI;
        double target_rpm = (velocity_rad_s / TWO_PI) * 60.0 * gear_ratio_;

        // 2. 변환된 RPM 값을 모터 드라이버에 전달합니다.
        if (joint_name == "wheel_left_joint")
        {
            int driver_idx = 0;
            int motor_idx = 0;

            if (driver_idx < num_driver)
            {
                // 변환된 RPM 값을 정수형으로 캐스팅하여 전달
                drivers[driver_idx]->motor_control_.target_vel[motor_idx] = static_cast<int32_t>(target_rpm);
                drivers[driver_idx]->motor_control_.set_vel_pos[motor_idx] = true;
            }
        }
        else if (joint_name == "wheel_right_joint")
        {
            int driver_idx = 0;
            int motor_idx = 1;

            if (driver_idx < num_driver)
            {
                // 변환된 RPM 값을 정수형으로 캐스팅하여 전달
                drivers[driver_idx]->motor_control_.target_vel[motor_idx] = static_cast<int32_t>(target_rpm);
                drivers[driver_idx]->motor_control_.set_vel_pos[motor_idx] = true;
            }
        }
    }
}

// JointState 발행 콜백 함수: 단위 변환 로직 포함
void DrivingMotorsCanOpen::publishJointStateCallback()
{
    if (!init_done_)
        return;

    auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->get_clock()->now();

    const double TWO_PI = 2.0 * M_PI;

    for (int i = 0; i < num_driver; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            // TODO: 발행되는 조인트 이름이 키네마틱스 노드와 일치하는지 확인하세요.
            // 현재는 "wheel_left_joint", "wheel_right_joint"로 가정합니다.
            if (j == 0) joint_state_msg->name.push_back("wheel_left_joint");
            else        joint_state_msg->name.push_back("wheel_right_joint");


            double raw_pos_ticks = drivers[i]->motor_status_.feedback_pos[j];
            double raw_vel_rpm = drivers[i]->motor_status_.feedback_vel[j];

            // 1. Position: Ticks -> Radians
            double position_rad = (raw_pos_ticks / ticks_per_revolution_) * TWO_PI / gear_ratio_;

            // 2. Velocity: RPM -> rad/s
            double velocity_rad_s = (raw_vel_rpm * TWO_PI / 60.0) / gear_ratio_;

            joint_state_msg->position.push_back(position_rad);
            joint_state_msg->velocity.push_back(velocity_rad_s);
            joint_state_msg->effort.push_back(0.0);
        }
    }

    joint_state_publisher_->publish(std::move(joint_state_msg));
}

void DrivingMotorsCanOpen::init()
{
    for (int j = 0; j < this->num_driver; j++)
    {
        for (int i = 0; i < 2; i++)
        {
            drivers[j]->motor_control_.set_acc_dec[i] = false;
            drivers[j]->motor_control_.set_vel_pos[i] = false;
        }
        drivers[j]->init_canopen = true;
    }
    logWrite(LOG_INFO, "init is done.");
    init_done_ = true;
}

void DrivingMotorsCanOpen::canOpenSysn()
{

    diag_set_handler(lelyLogHandler, nullptr);

    logWrite(LOG_INFO, "DrivingMotorsCanOpen::canOpenSysn starts.");
    io::IoGuard io_guard;
    io::Context ctx;
    io::Poll poll(ctx);
    ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    io::CanController ctrl(can_interface_.c_str());
    io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    logWrite(LOG_INFO, "Tcon_Master _master(timer,...");
    Tcon_Master _master(timer, chan, dcf_path_.c_str(), "", 111);
    master = &_master;

    std::stringstream ss;
    for (int i = 0; i < num_driver; ++i)
    {
        drivers_unique_.push_back(std::make_unique<VelocityControl>(exec, _master, i + 1));
        ss << "Initialized VelocityControl driver " << (i + 1);
    }
    logWrite(LOG_INFO, ss.str());

    for (int i = 0; i < num_driver; ++i)
    {
        drivers.push_back(drivers_unique_[i].get());
    }

    io::SignalSet sigset(poll, exec);
    sigset.insert(SIGHUP);
    sigset.insert(SIGINT);
    sigset.insert(SIGTERM);

    sigset.submit_wait([&](int /*signo*/)
                       {
                            // cout << "deconfiguration process:" << endl;
                            logWrite(LOG_INFO, "deconfiguration process start.");
                            sigset.clear();
                            master->AsyncDeconfig().submit(exec, [&]() {
                            ctx.shutdown();
                        }); });
    init();
    master->Reset();
    logWrite(LOG_INFO, "Master reset end.");

    loop.run();
}

bool DrivingMotorsCanOpen::getTimeOut(unsigned int dw_start_tick_count, unsigned int millisecound)
{
    if (elapseTime(dw_start_tick_count) >= millisecound)
        return true;
    else
        return false;
}

unsigned int DrivingMotorsCanOpen::getTickCount()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

unsigned long DrivingMotorsCanOpen::elapseTime(unsigned int dw_start_tick_count)
{
    return getTickCount() - dw_start_tick_count;
}

void DrivingMotorsCanOpen::logWrite(LogLevel level, std::string log_string)
{
    auto &logger = AmrLogger::getInstance();
    logger.logWrite(LOG_MOTOR_DRIVER, level, log_string);
}

std::vector<bool> DrivingMotorsCanOpen::intToBinaryVector(uint16_t num, uint8_t length)
{
    std::vector<bool> binaryVector(length, 0);
    int index = length - 1;
    while (num > 0 && index >= 0)
    {
        binaryVector[index] = num & 1;
        num >>= 1;
        index--;
    }
    std::reverse(binaryVector.begin(), binaryVector.end());
    return binaryVector;
}

void DrivingMotorsCanOpen::findErrorMessage(int code)
{
    for (const auto &error : errors)
    {
        if (error.error_code == code)
        {
            logWrite(LOG_INFO, "Error: " + error.error_name);
            return;
        }
    }
}