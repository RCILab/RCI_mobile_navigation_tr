#include "imu.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto imu_node = std::make_shared<Imu>();
    rclcpp::spin(imu_node);
    return 0;
}