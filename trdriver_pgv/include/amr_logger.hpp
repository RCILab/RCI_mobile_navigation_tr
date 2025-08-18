#ifndef AMR_LOGGER_HPP
#define AMR_LOGGER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <filesystem>

// 로그 레벨을 정의하는 열거형
enum LogLevel {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_FATAL
};

// 로그를 기록하는 소스(모듈)를 정의하는 열거형
// pgv.cpp에서 LOG_PGV를 사용하므로 여기에 정의합니다.
enum LogModule {
    LOG_GENERAL,
    LOG_PGV,
    LOG_IMU,
    LOG_NAV // 다른 모듈도 필요시 추가 가능
};

class AmrLogger
{
public:
    // 싱글톤 인스턴스를 얻기 위한 static 함수
    static AmrLogger& getInstance()
    {
        static AmrLogger instance; // C++11부터 thread-safe
        return instance;
    }

    // 로그를 파일에 쓰는 함수
    void logWrite(LogModule module, LogLevel level, const std::string& message)
    {
        std::lock_guard<std::mutex> guard(mtx_); // 멀티스레드 환경을 위한 잠금

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
    // 생성자를 private으로 선언하여 외부에서 인스턴스 생성을 막음
    AmrLogger() {
        // 로그 파일을 저장할 경로 설정 (사용자 홈 디렉토리의 tr_ws/log 폴더)
        std::string log_dir_path = std::string(getenv("HOME")) + "/tr_ws/log";
        
        // 로그 디렉토리가 없으면 생성
        if (!std::filesystem::exists(log_dir_path)) {
            std::filesystem::create_directories(log_dir_path);
        }

        std::string log_file_path = log_dir_path + "/pgv_log.txt";
        
        // 파일을 추가 모드(append)로 엽니다.
        log_file_.open(log_file_path, std::ios::out | std::ios::app);

        if (!log_file_.is_open()) {
            std::cerr << "Failed to open log file: " << log_file_path << std::endl;
        } else {
            logWrite(LOG_GENERAL, LOG_INFO, "Logger initialized.");
        }
    }

    // 소멸자에서 파일 닫기
    ~AmrLogger() {
        if (log_file_.is_open()) {
            logWrite(LOG_GENERAL, LOG_INFO, "Logger shutting down.");
            log_file_.close();
        }
    }

    // 복사 생성자와 대입 연산자를 삭제하여 싱글톤 패턴을 유지
    AmrLogger(const AmrLogger&) = delete;
    AmrLogger& operator=(const AmrLogger&) = delete;

    // 현재 시간을 문자열로 변환하는 헬퍼 함수
    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }

    // LogLevel 열거형을 문자열로 변환
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

    // LogModule 열거형을 문자열로 변환
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