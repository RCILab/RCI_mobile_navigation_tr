#include "driving_motor.hpp"
// #include "operator_motor.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto motors = std::make_shared<DrivingMotorsCanOpen>();
	// auto motors = std::make_shared<OperatorMotorsCanOpen>();

	executor.add_node(motors);

	executor.spin();

	rclcpp::shutdown();

	return 0;
}
