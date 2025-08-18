#ifndef AMR_LOGGER_HPP
#define AMR_LOGGER_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"

enum LogLevel {
    LOG_INFO,
    LOG_WARN,
    LOG_ERR
};

enum LogComponent {
    LOG_MOTOR_DRIVER
};

class AmrLogger {
public:
    static AmrLogger& getInstance() {
        static AmrLogger instance;
        return instance;
    }
    void logWrite(LogComponent component, LogLevel level, const std::string& log_string) {
        (void)component;
        auto logger = rclcpp::get_logger("tr_driver");
        switch(level) {
            case LOG_INFO:
                RCLCPP_INFO(logger, "%s", log_string.c_str());
                break;
            case LOG_WARN:
                RCLCPP_WARN(logger, "%s", log_string.c_str());
                break;
            case LOG_ERR:
                RCLCPP_ERROR(logger, "%s", log_string.c_str());
                break;
            default:
                break;
        }
    }
private:
    AmrLogger() = default;
};

#endif // AMR_LOGGER_HPP
