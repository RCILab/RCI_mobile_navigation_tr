#include "pgv.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto pgv_node = std::make_shared<PGV>();

    executor.add_node(pgv_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}