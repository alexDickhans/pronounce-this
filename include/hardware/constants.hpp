#pragma once

namespace Constants {

	QLength trackWidth = 19_in;
	QVelocity maxSpeed = 76.57632093_in / second;
	QAngularVelocity driveInputRpm = 600 * revolution / minute;

	std::unordered_map<uint16_t, pros::DeviceType> bothDevices({
		{18, pros::DeviceType::motor},
		{19, pros::DeviceType::motor},
		{20, pros::DeviceType::motor},
		{11, pros::DeviceType::motor},
		{12, pros::DeviceType::motor},
		{13, pros::DeviceType::motor},
		{2, pros::DeviceType::motor},
		{9, pros::DeviceType::radio},
		{14, pros::DeviceType::imu},});
	std::unordered_map<uint16_t, pros::DeviceType> skillsDevices({
		{1, pros::DeviceType::motor},
		{10, pros::DeviceType::motor},
		{8, pros::DeviceType::distance},});
	std::unordered_map<uint16_t, pros::DeviceType> matchDevices({
		{17, pros::DeviceType::motor},
		{16, pros::DeviceType::distance}});
}