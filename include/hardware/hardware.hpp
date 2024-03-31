#pragma once

#include "api.h"
#include "odometry/orientation/imu.hpp"
#include "pros/rtos.hpp"
#include "hardwareAbstractions/joystick/joystick.hpp"
#include "pros/apix.h"
#include <map>
#include "auton.h"
#include "units/units.hpp"
#include "constants.hpp"
#include "pros/motor_group.hpp"

#ifndef SIM

#include "hardwareAbstractions/joystick/robotJoystick.hpp"
#include "logger/logger.hpp"

#else
#include "hardwareAbstractions/joystick/simJoystick.hpp"
#endif // !SIM

namespace Pronounce {

	pros::Mutex robotMutex;

	constexpr bool isSkills = AUTON == 5;

#ifndef SIM
	RobotJoystick *master = new RobotJoystick(controller_id_e_t::E_CONTROLLER_MASTER);
#else
	AbstractJoystick* master = new SimJoystick(controller_id_e_t::E_CONTROLLER_MASTER);
#endif // !SIM

	pros::Mutex drivetrainMutex;

	pros::MotorGroup leftDriveMotors({-18, -19, -20}, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
	pros::MotorGroup rightDriveMotors({11, 12, 13}, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);

	TankDrivetrain drivetrain(Constants::trackWidth, Constants::maxSpeed, leftDriveMotors, rightDriveMotors,
	                          Constants::driveInputRpm);

	pros::MotorGroup catapultMotors({-1, 10}, pros::MotorGears::green, pros::MotorEncoderUnits::rotations);

	pros::MotorGroup winch({-2}, pros::v5::MotorGears::red, pros::MotorEncoderUnits::rotations);

	pros::adi::DigitalOut frontLeftSolenoid('G', false);
	pros::adi::DigitalOut frontRightSolenoid('F', false);
	pros::adi::DigitalOut backLeftSolenoid('D', false);
	pros::adi::DigitalOut backRightSolenoid('E', false);

	pros::MotorGroup intakeMotors({17}, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
	// Inertial Measurement Unit
	pros::Imu imu(14);
	IMU imuOrientation(imu);

	pros::Distance hopperDistanceSensor(16);
	pros::Distance catapultDistance(8);

	std::string checkPorts(const std::unordered_map<uint16_t, pros::DeviceType>& devices, std::string startString = "") {

		for (const auto &device: devices) {
			if (pros::c::registry_get_plugged_type(device.first-1) != static_cast<int>(device.second)) {
				if (startString.empty()) {
					startString.append(std::to_string(device.first));
				} else {
					startString.append(string_format(", %u", device.first));
				}
			}
		}

		return startString;
	}

	void initHardware() {
		Log("Hardware Init");
		leftDriveMotors.set_gearing_all(pros::MotorGears::blue);
		rightDriveMotors.set_gearing_all(pros::MotorGears::blue);
		leftDriveMotors.set_zero_position_all(0.0);
		rightDriveMotors.set_zero_position_all(0.0);

		drivetrain.setBrakeMode(pros::MotorBrake::coast);

		intakeMotors.set_brake_mode_all(pros::MotorBrake::coast);

		std::string portsReport = checkPorts(Constants::bothDevices);
		if (isSkills) {
			Log("Skills");
			portsReport = checkPorts(Constants::skillsDevices, portsReport);
		} else {
			Log("Match");
			portsReport = checkPorts(Constants::matchDevices, portsReport);
		}

		if (portsReport.empty()) {
			master->getController()->set_text(0, 0, "All good");
		} else {
			master->getController()->set_text(0, 0, portsReport.c_str());
			pros::Task::delay(50);
			master->getController()->rumble("-.-.-.-.");
		}

		Log(string_format("Ports missing: %s", portsReport.c_str()));

		if ((imu.is_installed() && pros::competition::is_disabled()) || isSkills) {
			imu.reset();
			Log("Imu: calibrate");

			while (imu.is_calibrating())
				pros::Task::delay(50);

			Log("Imu: done calibrating");
		} else {
			Log("ERROR Imu: Not installed")
		}

		Log("Hardware Init Done");
	}
} // namespace Pronounce
