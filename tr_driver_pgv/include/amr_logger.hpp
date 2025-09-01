#ifndef AMR_LOGGER_HPP
#define AMR_LOGGER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <filesystem>

enum LogLevel {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_FATAL
};

enum LogModule {
    LOG_GENERAL,
    LOG_PGV,
    LOG_IMU,
    LOG_NAV
};

class AmrLogger
{
public:
    static AmrLogger& getInstance()
    {
        static AmrLogger instance;
        return instance;
    }

    void logWrite(LogModule module, LogLevel level, const std::string& message)
    {
        std::lock_guard<std::mutex> guard(mtx_);
        if (!log_file_.is_open()) {
            std::cerr << "Logger file is not open!" << std::endl;
            return;
        }

        log_file_ << getCurrentTimestamp()
                  << " " << moduleToString(module)
                  << " " << levelToString(level)
                  << ": " << message << std::endl;
    }

private:
    AmrLogger() {
        std::string log_dir_path = std::string(getenv("HOME")) + "/tr_ws/log";

        if (!std::filesystem::exists(log_dir_path)) {
            std::filesystem::create_directories(log_dir_path);
        }

        std::string log_file_path = log_dir_path + "/pgv_log.txt";

        log_file_.open(log_file_path, std::ios::out | std::ios::app);

        if (!log_file_.is_open()) {
            std::cerr << "Failed to open log file: " << log_file_path << std::endl;
        } else {
            logWrite(LOG_GENERAL, LOG_INFO, "Logger initialized.");
        }
    }

    ~AmrLogger() {
        if (log_file_.is_open()) {
            logWrite(LOG_GENERAL, LOG_INFO, "Logger shutting down.");
            log_file_.close();
        }
    }

    AmrLogger(const AmrLogger&) = delete;
    AmrLogger& operator=(const AmrLogger&) = delete;

    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }

    std::string levelToString(LogLevel level) {
        switch (level) {
            case LOG_DEBUG: return "[DEBUG]";
            case LOG_INFO:  return "[INFO]";
            case LOG_WARN:  return "[WARN]";
            case LOG_ERROR: return "[ERROR]";
            case LOG_FATAL: return "[FATAL]";
            default:        return "[UNKNOWN]";
        }
    }

    std::string moduleToString(LogModule module) {
        switch (module) {
            case LOG_GENERAL: return "[GENERAL]";
            case LOG_PGV:     return "[PGV]";
            case LOG_IMU:     return "[IMU]";
            case LOG_NAV:     return "[NAV]";
            default:          return "[UNKNOWN_MODULE]";
        }
    }

    std::ofstream log_file_;
    std::mutex mtx_;
};

#endif // AMR_LOGGER_HPP