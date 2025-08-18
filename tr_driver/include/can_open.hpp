#ifndef CAN_OPEN_HPP
#define CAN_OPEN_HPP

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/sdo.hpp>

#include "data_type/constants.h"
#include "data_type/alarm_data.hpp"
#include "helper/thread_safety_queue.hpp"
#include "helper/converter.hpp"
#include "helper/amr_logger.hpp"
#include "helper/file_io.hpp"
#include "motor_define.hpp"

#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <thread>
#include <bitset>
#include <unistd.h>
#include <sched.h>
#include <math.h>
#include <variant>

using namespace std;
using namespace std::chrono_literals;
using namespace lely;

struct MotorStatus
{
	int32_t feedback_vel[2] = {0, 0};
	int32_t feedback_pos[2] = {0, 0};
	bool digital_output[2][4] = {{0, 0, 0, 0},
								 {0, 0, 0, 0}};
	uint16_t status_flag = 0;
	bool status[2][8] = {{0, 0, 0, 0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0, 0, 0, 0}};
	bool fault[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint16_t fault_flag = 0;
	bool forward_limit[2] = {false, false};
	bool reverse_limit[2] = {false, false};
	bool home_comp[2] = {false, false};
	bool homing[2] = {false, false};
	bool in_home[2] = {false, false};
	bool motor_busy[2] = {false, false};
	int16_t amps[2] = {0, 0};
	uint16_t alarm_code;
	int32_t var1 = 0;
	int32_t var2 = 0;
	int32_t var3 = 0;
	int32_t var4 = 0;
	int32_t var5 = 0;
	int32_t var6 = 0;
	int32_t var7 = 0;
	int32_t var8 = 0;
};

struct MotorControl
{
	int32_t target_vel[2] = {0, 0};
	int32_t target_pos[2] = {0, 0};
	uint16_t acceleration[2] = {0, 0};
	uint16_t deceleration[2] = {0, 0};
	int32_t var9 = 0;
	int32_t var10 = 0;
	int32_t var11 = 0;
	int32_t var12 = 0;
	int32_t var13 = 0;
	int32_t var14 = 0;
	int32_t var15 = 0;
	int32_t var16 = 0;
	bool set_acc_dec[2] = {false, false};
	bool set_vel_pos[2] = {false, false};
	uint8_t operation_mode[2] = {0, 0};
};

// Define a visitor that converts the variant to a string
struct VariantToStringVisitor
{
	std::string operator()(int8_t v) const { return std::to_string(v); }
	std::string operator()(uint8_t v) const { return std::to_string(v); }
	std::string operator()(int16_t v) const { return std::to_string(v); }
	std::string operator()(uint16_t v) const { return std::to_string(v); }
	std::string operator()(int32_t v) const { return std::to_string(v); }
	std::string operator()(uint32_t v) const { return std::to_string(v); }
	std::string operator()(double v) const { return std::to_string(v); }
	std::string operator()(const std::string &v) const { return v; }
};

class VelocityControl : public canopen::FiberDriver
{
public:
	using FiberDriver::FiberDriver;

	// VelocityControl(const VelocityControl &) = default;			   // Enable copy constructor
	// VelocityControl &operator=(const VelocityControl &) = default; // Enable copy assignment

public:
	MotorStatus motor_status_;
	MotorControl motor_control_;
	error_code write_sdo_err;

	bool comm_fail_ = false;
	bool send_vel_ = false;

	bool turn_off_motor_node_ = false;
	bool error_allmt = false;
	bool home_finish_all = false;

	bool write_SDO_ = false;
	uint8_t count_SDO_ = 0;
	uint16_t idx_SDO_ = 0;
	uint8_t subidx_SDO_ = 0;
	uint8_t count_disconnect_ = 0;
	int32_t comm_signal_ = 0;
	bool init_canopen = false;

private:
	// Using std::variant to store different types of values
	std::variant<int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, double, std::string> value_SDO_;
	bool is_first_pos_received_{false};
	int32_t position_offset_[2]{0, 0};

private:
	void OnNodeGuarding(bool /* occurred */) noexcept override
	{
		logWrite(LOG_INFO, "Motor ID " + std::to_string(static_cast<int>(id())) + " node guarding fail.");
		comm_fail_ = true;
	}

	void OnHeartbeat(bool /*  occurred */) noexcept override
	{
		logWrite(LOG_INFO, "Motor ID " + std::to_string(static_cast<int>(id())) + " heartbeat fail.");
		comm_fail_ = true;
	}

	// function run with cycle time 10ms
	void OnSync(uint8_t /* cnt */, const time_point & /* t */) noexcept override
	{
		if (init_canopen)
		{
			count_disconnect_ = count_disconnect_ + 1;
			if (count_disconnect_ == 10)
			{
				comm_signal_ = comm_signal_ + 1;
				tpdo_mapped[0x2005][15] = comm_signal_;
				tpdo_mapped[0x2005][15].WriteEvent();
				count_disconnect_ = 0;
				if (comm_signal_ > 200000)
					comm_signal_ = 0;
			}

			// if (motor_control_.set_acc_dec[0] || motor_control_.set_acc_dec[1])
			// {
			// 	tpdo_mapped[0x2005][9] = motor_control_.acceleration[0] + (motor_control_.deceleration[0] << 16);
			// 	tpdo_mapped[0x2005][10] = motor_control_.acceleration[1] + (motor_control_.deceleration[1] << 16);
			// 	tpdo_mapped[0x2005][9].WriteEvent();
			// 	if (motor_control_.set_acc_dec[0])
			// 		motor_control_.set_acc_dec[0] = false;
			// 	if (motor_control_.set_acc_dec[1])
			// 		motor_control_.set_acc_dec[1] = false;
			// }
			// // use else if to send acc dec before send velocity and position
			// else if (motor_control_.set_vel_pos[0])
			// {
			// 	tpdo_mapped[0x2005][11] = motor_control_.target_vel[0];
			// 	tpdo_mapped[0x2005][12] = motor_control_.target_pos[0];
			// 	tpdo_mapped[0x2005][11].WriteEvent();
			// 	motor_control_.set_vel_pos[0] = false;
			// }
			// // use else if to send acc dec before send velocity and position
			// else if (motor_control_.set_vel_pos[1])
			// {
			// 	tpdo_mapped[0x2005][13] = motor_control_.target_vel[1];
			// 	tpdo_mapped[0x2005][14] = motor_control_.target_pos[1];
			// 	tpdo_mapped[0x2005][13].WriteEvent();
			// 	motor_control_.set_vel_pos[1] = false;
			// }

			if (motor_control_.set_acc_dec[0] || motor_control_.set_acc_dec[1])
			{
				tpdo_mapped[0x2005][9] = motor_control_.acceleration[0] + (motor_control_.deceleration[0] << 16);
				tpdo_mapped[0x2005][10] = motor_control_.acceleration[1] + (motor_control_.deceleration[1] << 16);
				tpdo_mapped[0x2005][9].WriteEvent();
				if (motor_control_.set_acc_dec[0])
					motor_control_.set_acc_dec[0] = false;
				if (motor_control_.set_acc_dec[1])
					motor_control_.set_acc_dec[1] = false;
			}
			// use else if to send acc dec before send velocity and position
			if (motor_control_.set_vel_pos[0])
			{
				tpdo_mapped[0x2005][11] = motor_control_.target_vel[0];
				tpdo_mapped[0x2005][12] = motor_control_.target_pos[0];
				tpdo_mapped[0x2005][11].WriteEvent();
				motor_control_.set_vel_pos[0] = false;
			}
			// use else if to send acc dec before send velocity and position
			if (motor_control_.set_vel_pos[1])
			{
				tpdo_mapped[0x2005][13] = motor_control_.target_vel[1];
				tpdo_mapped[0x2005][14] = motor_control_.target_pos[1];
				tpdo_mapped[0x2005][13].WriteEvent();
				motor_control_.set_vel_pos[1] = false;
			}

			if (write_SDO_)
			{
				std::visit([&](auto &&val)
						   { WriteSDO(idx_SDO_, subidx_SDO_, val); }, value_SDO_);
				write_SDO_ = false;
			}
		}
	}

	void OnBoot(canopen::NmtState /*st*/, char es, const std::string &what) noexcept override
	{
		if (!es || es == 'L')
			logWrite(LOG_INFO, "Motor ID " + std::to_string(static_cast<int>(id())) + " booted sucessfully");
		else
			logWrite(LOG_INFO, "Motor ID " + std::to_string(static_cast<int>(id())) + " failed to boot: " + what);
	}

	void OnConfig(std::function<void(std::error_code ec)> res) noexcept override
	{
		try
		{
			logWrite(LOG_INFO, "Set operation mode Walk motor: 3 velocity mode");
			// Wait(AsyncWrite(0x3027, 1, 3), write_sdo_err);

			// Configure the slave to monitor the heartbeat of the master (node-ID 111)
			// with a timeout of 200 ms.
			// logWrite(LOG_INFO, "heartbeat 0x1016");
			Wait(AsyncWrite<uint32_t>(0x1016, 1, 300), write_sdo_err);
			// Configure the slave to produce a heartbeat every 200 ms.
			// logWrite(LOG_INFO, "heartbeat 0x1017");
			// Wait(AsyncWrite<uint16_t>(0x1017, 0, 200), write_sdo_err);
			// Configure the heartbeat consumer on the master.
			// logWrite(LOG_INFO, "Configure the heartbeat consumer on the master.");
			ConfigHeartbeat(600ms);

			// // Set guarding time
			// Wait(AsyncWrite(0x100C, 0, (uint16_t)200), write_sdo_err);
			// // Set life time factor
			// Wait(AsyncWrite(0x100D, 0, (uint8_t)2), write_sdo_err);
			comm_fail_ = false;
			res({}); // Report success (empty error code).
		}
		catch (canopen::SdoError &e)
		{
			res(e.code()); // If SDO error, abort the configuration and report the error code.
			std::cout << "OnConfig VelocityControl " << unsigned(id()) << " ERROR" << std::endl;
			comm_fail_ = true;
		}
	}

	void OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override
	{
		try
		{
			// Disable the driver
			turn_off_motor_node_ = true;

			// Wait(AsyncWrite<int32_t>(0x2005, 11, 0));

			// Wait(AsyncWrite<int32_t>(0x2005, 15, 0));

			// Wait(AsyncWrite<int32_t>(0x2005, 11, 0));

			// Wait(AsyncWrite<int32_t>(0x2005, 15, 0));

			// // Disable the heartbeat producer on the slave.
			// Wait(AsyncWrite<uint16_t>(0x1017, 0, 0));
			// // // Disable the heartbeat consumer on the slave.
			// Wait(AsyncWrite<uint32_t>(0x1016, 1, 0));
			// // Set guarding time
			// Wait(AsyncWrite<uint16_t>(0x100C, 0, 0));
			// // Set life time factor
			// Wait(AsyncWrite<uint16_t>(0x100D, 0, 0));

			// Disable the heartbeat consumer on the master.
			ConfigHeartbeat(0ms);
			res({});
		}
		catch (canopen::SdoError &e)
		{
			res(e.code());
		}
	}

	void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override
	{
		comm_fail_ = false;
		switch (idx)
		{
		case 0x2106:
			if (subidx == 1 || subidx == 2)
			{
				if (!is_first_pos_received_)
				{
					position_offset_[0] = rpdo_mapped[0x2106][1];
					position_offset_[1] = rpdo_mapped[0x2106][2];
					is_first_pos_received_ = true;
				}
				motor_status_.feedback_pos[0] = (int32_t)rpdo_mapped[0x2106][1] - position_offset_[0];
				motor_status_.feedback_pos[1] = (int32_t)rpdo_mapped[0x2106][2] - position_offset_[1];
			}
			if (subidx == 3)
			{
				motor_status_.feedback_vel[0] = (((int32_t)rpdo_mapped[0x2106][3]) & 0xFFFF) - 5000;
				motor_status_.feedback_vel[1] = ((((int32_t)rpdo_mapped[0x2106][3]) >> 16) & 0xFFFF) - 5000;
			}
			if (subidx == 4)
			{
				motor_status_.homing[0] = ((int32_t)rpdo_mapped[0x2106][4]) & 0x1;
				motor_status_.home_comp[0] = (((int32_t)rpdo_mapped[0x2106][4]) >> 1) & 0x1;
				motor_status_.motor_busy[0] = (((int32_t)rpdo_mapped[0x2106][4]) >> 2) & 0x1;
				motor_status_.homing[1] = (((int32_t)rpdo_mapped[0x2106][4]) >> 3) & 0x1;
				motor_status_.home_comp[1] = (((int32_t)rpdo_mapped[0x2106][4]) >> 4) & 0x1;
				motor_status_.motor_busy[1] = (((int32_t)rpdo_mapped[0x2106][4]) >> 5) & 0x1;

				std::vector<bool> temp_vector(9);
				uint16_t temp_fault = (((int32_t)rpdo_mapped[0x2106][4]) >> 16) & 0xFFFF;
				motor_status_.fault_flag = temp_fault;
				temp_vector = intToBinaryVector(temp_fault, 9);
				for (std::vector<int>::size_type i = 0; i < temp_vector.size(); i++)
				{
					motor_status_.fault[i] = temp_vector[i];
				}
			}
			if (subidx == 5)
			{
				std::vector<bool> temp_vector(8);
				uint16_t temp_status_1 = ((int32_t)rpdo_mapped[0x2106][5]) & 0xFFFF;
				temp_vector = intToBinaryVector(temp_status_1, 8);
				for (std::vector<int>::size_type i = 0; i < temp_vector.size(); i++)
				{
					motor_status_.status[0][i] = temp_vector[i];
				}

				uint16_t temp_status_2 = (((int32_t)rpdo_mapped[0x2106][5]) >> 16) & 0xFFFF;
				temp_vector = intToBinaryVector(temp_status_2, 8);
				for (std::vector<int>::size_type i = 0; i < temp_vector.size(); i++)
				{
					motor_status_.status[1][i] = temp_vector[i];
				}
			}
			if (subidx == 6)
			{
				motor_status_.amps[0] = (((int32_t)rpdo_mapped[0x2106][6]) & 0xFFFF) - 5000;
				motor_status_.amps[1] = ((((int32_t)rpdo_mapped[0x2106][6]) >> 16) & 0xFFFF) - 5000;
			}
			break;
		}
	}

public:
	// Templated setter method
	template <typename T>
	void setValue(T val)
	{
		value_SDO_ = val;
		// std::string value_str = std::visit(VariantToStringVisitor{}, value_SDO_);
		// logWrite(LOG_INFO, "value_SDO_: " + value_str);
	}

	template <typename T>
	bool WriteSDO(uint16_t idx, uint8_t subidx, T value)
	{
		try
		{
			if constexpr (std::is_same_v<T, int8_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<int8_t>(value)), write_sdo_err);
			}
			else if constexpr (std::is_same_v<T, uint8_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<uint8_t>(value)), write_sdo_err);
			}
			else if constexpr (std::is_same_v<T, int16_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<int16_t>(value)), write_sdo_err);
			}
			else if constexpr (std::is_same_v<T, uint16_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<uint16_t>(value)), write_sdo_err);
			}
			else if constexpr (std::is_same_v<T, int32_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<int32_t>(value)), write_sdo_err);
			}
			else if constexpr (std::is_same_v<T, uint32_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<uint32_t>(value)), write_sdo_err);
			}
			else
			{
				throw std::invalid_argument("Unsupported type for WriteSDO");
			}

			// Convert the variant to string using the visitor
			std::string value_str = std::visit(VariantToStringVisitor{}, value_SDO_);

			write_sdo_err = std::error_code();
			if (write_sdo_err)
			{
				std::stringstream ss;
				ss << std::showbase << std::hex << idx;
				std::string hex_string = ss.str(); // Convert to string
				logWrite(LOG_ERR, "WriteSDO: Motor ID " + std::to_string(static_cast<int>(id())) +
									  ", address " + hex_string + ", subindex " + std::to_string(subidx) +
									  ", value " + value_str + " FAIL");
				return false;
			}
			else
			{
				std::stringstream ss;
				ss << std::showbase << std::hex << idx;
				std::string hex_string = ss.str(); // Convert to string
				logWrite(LOG_INFO, "WriteSDO: Motor ID " + std::to_string(static_cast<int>(id())) +
									   ", address " + hex_string + ", subindex " + std::to_string(subidx) +
									   ", value " + value_str + " SUCCESS");
				return true;
			}
		}
		catch (const std::exception &e)
		{
			logWrite(LOG_ERR, "WriteSDO failed: " + std::string(e.what()));
			return false;
		}
	}

	// convert int to binary vector
	std::vector<bool> intToBinaryVector(uint16_t num, uint8_t length)
	{
		std::vector<bool> binaryVector(length, 0);
		// Start from the end of the vector
		int index = length - 1;
		// Continue shifting the bits until num becomes 0
		while (num > 0 && index >= 0)
		{
			// Extract the least significant bit and assign it to the vector
			binaryVector[index] = num & 1;
			// Right shift num by 1 bit
			num >>= 1;
			// Move to the previous position in the vector
			index--;
		}
		// Reverse the vector to get the correct binary representation
		std::reverse(binaryVector.begin(), binaryVector.end());
		return binaryVector;
	}

	void logWrite(LogLevel level, std::string log_string)
	{
		auto &logger = AmrLogger::getInstance();
		logger.logWrite(LOG_MOTOR_DRIVER, level, log_string);
	}
};

class PositionControl : public canopen::FiberDriver
{
public:
	using FiberDriver::FiberDriver;

public:
	MotorStatus motor_status_;
	MotorControl motor_control_;
	error_code write_sdo_err;

	bool comm_fail_ = false;
	bool send_vel_ = false;

	bool turn_off_motor_node_ = false;
	bool error_allmt = false;
	bool home_finish_all = false;

	bool write_SDO_ = false;
	uint8_t count_SDO_ = 0;
	uint16_t idx_SDO_ = 0;
	uint8_t subidx_SDO_ = 0;

private:
	// Using std::variant to store different types of values
	std::variant<int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, double, std::string> value_SDO_;
	bool is_first_pos_received_{false};
	int32_t position_offset_[2]{0, 0};

private:
	void OnNodeGuarding(bool /* occurred */) noexcept override
	{
		logWrite(LOG_INFO, "Motor ID " + std::to_string(static_cast<int>(id())) + " node guarding fail.");
		comm_fail_ = true;
	}

	// function run with cycle time 10ms
	void OnSync(uint8_t /* cnt */, const time_point & /* t */) noexcept override
	{
		if (motor_control_.set_acc_dec[0])
		{
			tpdo_mapped[0x2005][9] = motor_control_.acceleration[0];
			tpdo_mapped[0x2005][10] = motor_control_.deceleration[0];
			tpdo_mapped[0x2005][9].WriteEvent();
			motor_control_.set_acc_dec[0] = false;
		}
		// use else if to send acc dec before send velocity and position
		else if (motor_control_.set_vel_pos[0])
		{
			tpdo_mapped[0x2005][11] = motor_control_.target_vel[0];
			tpdo_mapped[0x2005][12] = motor_control_.target_pos[0];
			tpdo_mapped[0x2005][11].WriteEvent();
			motor_control_.set_vel_pos[0] = false;
		}

		if (motor_control_.set_acc_dec[1])
		{
			tpdo_mapped[0x2005][13] = motor_control_.acceleration[1];
			tpdo_mapped[0x2005][14] = motor_control_.deceleration[1];
			tpdo_mapped[0x2005][13].WriteEvent();
			motor_control_.set_acc_dec[1] = false;
		}
		// use else if to send acc dec before send velocity and position
		else if (motor_control_.set_vel_pos[1])
		{
			tpdo_mapped[0x2005][15] = motor_control_.target_vel[1];
			tpdo_mapped[0x2005][16] = motor_control_.target_pos[1];
			tpdo_mapped[0x2005][15].WriteEvent();
			motor_control_.set_vel_pos[1] = false;
		}

		if (write_SDO_)
		{
			std::visit([&](auto &&val)
					   { WriteSDO(idx_SDO_, subidx_SDO_, val); }, value_SDO_);
			write_SDO_ = false;
			// count_SDO_ = count_SDO_ + 1;
			// if (count_SDO_ == 4)
			// {
			// 	write_SDO_ = false;
			// 	count_SDO_ = 0;
			// }
		}
	}

	void OnBoot(canopen::NmtState /*st*/, char es, const std::string &what) noexcept override
	{
		if (!es || es == 'L')
			logWrite(LOG_INFO, "Motor ID " + std::to_string(static_cast<int>(id())) + " booted sucessfully");
		else
			logWrite(LOG_INFO, "Motor ID " + std::to_string(static_cast<int>(id())) + " failed to boot: " + what);
	}

	void OnConfig(std::function<void(std::error_code ec)> res) noexcept override
	{
		try
		{
			logWrite(LOG_INFO, "Set operation mode Walk motor: 3 velocity mode");
			Wait(AsyncWrite(0x3027, 1, 3), write_sdo_err);

			// // Configure the slave to monitor the heartbeat of the master (node-ID 111)
			// // with a timeout of 600 ms.
			// Wait(AsyncWrite<uint32_t>(0x1016, 1, (5 << 16) | 600));
			// // Configure the slave to produce a heartbeat every 300 ms.
			// Wait(AsyncWrite(0x1017, 0, (uint16_t)300), write_sdo_err);
			// // Configure the heartbeat consumer on the master.
			// ConfigHeartbeat(600ms);

			// Set guarding time
			Wait(AsyncWrite(0x100C, 0, (uint16_t)200), write_sdo_err);
			// Set life time factor
			Wait(AsyncWrite(0x100D, 0, (uint8_t)2), write_sdo_err);
			comm_fail_ = false;
			res({}); // Report success (empty error code).
		}
		catch (canopen::SdoError &e)
		{
			res(e.code()); // If SDO error, abort the configuration and report the error code.
			std::cout << "OnConfig VelocityControl " << unsigned(id()) << " ERROR" << std::endl;
			comm_fail_ = true;
		}
	}

	void OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override
	{
		try
		{
			// Disable the driver
			turn_off_motor_node_ = true;
			// Disable the heartbeat consumer on the master.
			ConfigHeartbeat(0ms);
			// Disable the heartbeat producer on the slave.
			Wait(AsyncWrite<uint16_t>(0x1017, 0, 0));
			// // Disable the heartbeat consumer on the slave.
			Wait(AsyncWrite<uint32_t>(0x1016, 1, 0));
			// Set guarding time
			Wait(AsyncWrite<uint16_t>(0x100C, 0, 0));
			// Set life time factor
			Wait(AsyncWrite<uint16_t>(0x100D, 0, 0));
			res({});
		}
		catch (canopen::SdoError &e)
		{
			res(e.code());
		}
	}

	void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override
	{
		comm_fail_ = false;
		switch (idx)
		{
		case 0x2106:
			if (subidx == 1 || subidx == 2)
			{
				if (!is_first_pos_received_)
				{
					position_offset_[0] = rpdo_mapped[0x2106][1];
					position_offset_[1] = rpdo_mapped[0x2106][2];
					is_first_pos_received_ = true;
				}
				motor_status_.feedback_pos[0] = (int32_t)rpdo_mapped[0x2106][1] - position_offset_[0];
				motor_status_.feedback_pos[1] = (int32_t)rpdo_mapped[0x2106][2] - position_offset_[1];
			}
			if (subidx == 3)
				motor_status_.feedback_vel[0] = rpdo_mapped[0x2106][3];
			if (subidx == 4)
				motor_status_.feedback_vel[1] = rpdo_mapped[0x2106][4];
			if (subidx == 5)
			{
				std::vector<bool> temp_vector(8);
				uint16_t temp_status_1 = ((int32_t)rpdo_mapped[0x2106][5]) & 0xFFFF;
				temp_vector = intToBinaryVector(temp_status_1, 8);
				for (std::vector<int>::size_type i = 0; i < temp_vector.size(); i++)
				{
					motor_status_.status[0][i] = temp_vector[i];
				}

				uint16_t temp_status_2 = (((int32_t)rpdo_mapped[0x2106][5]) >> 16) & 0xFFFF;
				temp_vector = intToBinaryVector(temp_status_2, 8);
				for (std::vector<int>::size_type i = 0; i < temp_vector.size(); i++)
				{
					motor_status_.status[1][i] = temp_vector[i];
				}
			}
			if (subidx == 6)
			{
				motor_status_.homing[0] = ((int32_t)rpdo_mapped[0x2106][6]) & 0x1;
				motor_status_.home_comp[0] = (((int32_t)rpdo_mapped[0x2106][6]) >> 1) & 0x1;
				motor_status_.motor_busy[0] = (((int32_t)rpdo_mapped[0x2106][6]) >> 2) & 0x1;
				motor_status_.homing[1] = (((int32_t)rpdo_mapped[0x2106][6]) >> 3) & 0x1;
				motor_status_.home_comp[1] = (((int32_t)rpdo_mapped[0x2106][6]) >> 4) & 0x1;
				motor_status_.motor_busy[1] = (((int32_t)rpdo_mapped[0x2106][6]) >> 5) & 0x1;

				std::vector<bool> temp_vector(9);
				uint16_t temp_fault = (((int32_t)rpdo_mapped[0x2106][6]) >> 16) & 0xFFFF;
				motor_status_.fault_flag = temp_fault;
				temp_vector = intToBinaryVector(temp_fault, 9);
				for (std::vector<int>::size_type i = 0; i < temp_vector.size(); i++)
				{
					motor_status_.fault[i] = temp_vector[i];
				}
			}
			if (subidx == 7)
				motor_status_.amps[0] = ((int32_t)rpdo_mapped[0x2106][7]) & 0xFFFF;
			if (subidx == 8)
				motor_status_.amps[1] = ((int32_t)rpdo_mapped[0x2106][8]) & 0xFFFF;
			break;
		}
	}

public:
	// Templated setter method
	template <typename T>
	void setValue(T val)
	{
		value_SDO_ = val;
		// std::string value_str = std::visit(VariantToStringVisitor{}, value_SDO_);
		// logWrite(LOG_INFO, "value_SDO_: " + value_str);
	}

	template <typename T>
	bool WriteSDO(uint16_t idx, uint8_t subidx, T value)
	{
		try
		{
			if constexpr (std::is_same_v<T, int8_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<int8_t>(value)), write_sdo_err);
			}
			else if constexpr (std::is_same_v<T, uint8_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<uint8_t>(value)), write_sdo_err);
			}
			else if constexpr (std::is_same_v<T, int16_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<int16_t>(value)), write_sdo_err);
			}
			else if constexpr (std::is_same_v<T, uint16_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<uint16_t>(value)), write_sdo_err);
			}
			else if constexpr (std::is_same_v<T, int32_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<int32_t>(value)), write_sdo_err);
			}
			else if constexpr (std::is_same_v<T, uint32_t>)
			{
				Wait(AsyncWrite(idx, subidx, static_cast<uint32_t>(value)), write_sdo_err);
			}
			else
			{
				throw std::invalid_argument("Unsupported type for WriteSDO");
			}

			// Convert the variant to string using the visitor
			std::string value_str = std::visit(VariantToStringVisitor{}, value_SDO_);

			write_sdo_err = std::error_code();
			if (write_sdo_err)
			{
				std::stringstream ss;
				ss << std::showbase << std::hex << idx;
				std::string hex_string = ss.str(); // Convert to string
				logWrite(LOG_ERR, "WriteSDO: Motor ID " + std::to_string(static_cast<int>(id())) +
									  ", address " + hex_string + ", subindex " + std::to_string(subidx) +
									  ", value " + value_str + " FAIL");
				return false;
			}
			else
			{
				std::stringstream ss;
				ss << std::showbase << std::hex << idx;
				std::string hex_string = ss.str(); // Convert to string
				logWrite(LOG_INFO, "WriteSDO: Motor ID " + std::to_string(static_cast<int>(id())) +
									   ", address " + hex_string + ", subindex " + std::to_string(subidx) +
									   ", value " + value_str + " SUCCESS");
				return true;
			}
		}
		catch (const std::exception &e)
		{
			logWrite(LOG_ERR, "WriteSDO failed: " + std::string(e.what()));
			return false;
		}
	}

	// convert int to binary vector
	std::vector<bool> intToBinaryVector(uint16_t num, uint8_t length)
	{
		std::vector<bool> binaryVector(length, 0);
		// Start from the end of the vector
		int index = length - 1;
		// Continue shifting the bits until num becomes 0
		while (num > 0 && index >= 0)
		{
			// Extract the least significant bit and assign it to the vector
			binaryVector[index] = num & 1;
			// Right shift num by 1 bit
			num >>= 1;
			// Move to the previous position in the vector
			index--;
		}
		// Reverse the vector to get the correct binary representation
		std::reverse(binaryVector.begin(), binaryVector.end());
		return binaryVector;
	}

	void logWrite(LogLevel level, std::string log_string)
	{
		auto &logger = AmrLogger::getInstance();
		logger.logWrite(LOG_MOTOR_DRIVER, level, log_string);
	}
};

// Master class
class Tcon_Master : public canopen::AsyncMaster
{
public:
	using AsyncMaster::AsyncMaster;
	bool comm_fail_1 = false;

private:
};

#endif