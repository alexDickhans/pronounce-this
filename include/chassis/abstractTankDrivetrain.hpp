#pragma once

#include "abstractDrivetrain.hpp"
#include "utils/utils.hpp"
#include "units/units.hpp"
#include <iostream>

namespace Pronounce {

	/**
	 * @brief Abstract tank drivetrain class for all tank drivetrains
	 * 
	 * @authors Alex Dickhans(ad101-lab)
	 */
	class AbstractTankDrivetrain : public AbstractDrivetrain {
	private:
		/**
		 * @brief The distance between the two drive sides
		 * 
		 * @details Used for driveCurvature and other math things
		 * 
		 */
		QLength trackWidth;

		/**
		 * @brief Max speed of the drivetrain
		 * 
		 */
		QSpeed maxSpeed;
	public:
		/**
		 * @brief Construct a new Abstract Tank Drivetrain object with all values set to zero. 
		 * Doing any functions that require the track width will require the value set
		 * 
		 */
		AbstractTankDrivetrain() : trackWidth(0.0) {}

		/**
		 * @brief Construct a new Abstract Tank Drivetrain object with trackWidth set
		 * 
		 * @param trackWidth The trackWidth of the drivetrain
		 */
		AbstractTankDrivetrain(QLength trackWidth, QSpeed maxSpeed) : trackWidth(trackWidth) {}

		/**
		 * @brief Get the current speed of the robot
		 * 
		 * @return QSpeed The current speed of the robot
		 */
		virtual QSpeed getSpeed() { return 0.0; }

		/**
		 * @brief Get the Track Width distance 
		 * 
		 * @attention Requires trackWidth to be set
		 * 
		 * @return double The distance between the two sides of the drivetrain
		 */
		QLength getTrackWidth() {
			return trackWidth;
		}

		/**
		 * @brief Set the Track Width distance
		 * 
		 * @param trackWidth The distance between the two sides of the drivetrain
		 */
		void setTrackWidth(QLength trackWidth) {
			this->trackWidth = trackWidth;
		}

		/**
		 * @brief Drives the tank drivetrain at a specified curvature and speed value
		 * 
		 * @attention Requires trackWidth to be set
		 * 
		 * @param speed The desired speed specified by QSpeed
		 * @param curvature The desired curvature given by QCurvature
		 */
		void driveCurvature(QSpeed speed, QCurvature curvature) {			
			QSpeed leftSpeed = speed.getValue() * (2.0 + curvature.getValue() * trackWidth.getValue()) / 2.0;
			QSpeed rightSpeed = speed.getValue() * (2.0 - curvature.getValue() * trackWidth.getValue()) / 2.0;

			QSpeed maxSpeed = max(fabs(leftSpeed.getValue()), fabs(rightSpeed.getValue()));

			if (maxSpeed.getValue() > fabs(speed.getValue())) {
				double multiplier = fabs(speed.getValue()) / maxSpeed.getValue();
				leftSpeed = leftSpeed.getValue() * multiplier;
				rightSpeed = rightSpeed.getValue() * multiplier;
			}

			this->tankSteerVelocity(leftSpeed, rightSpeed);
		}

		/**
		 * @brief Drives the tank drivetrain at a specified curvature and voltage value
		 * 
		 * @attention Requires trackWidth to be set
		 * 
		 * @param voltage The desired max voltage of the drivetrain
		 * @param curvature The desired curvature of the drivetrain
		 */
		void driveCurvatureVoltage(double voltage, QCurvature curvature) {
			double leftVoltage = voltage * (2.0 + curvature.getValue() * trackWidth.getValue()) / 2.0;
			double rightVoltage = voltage * (2.0 - curvature.getValue() * trackWidth.getValue()) / 2.0;

			double maxVoltage = max(fabs(leftVoltage), fabs(rightVoltage));

			if (maxVoltage > fabs(voltage)) {
				double multiplier = fabs(voltage) / maxVoltage;
				leftVoltage *= multiplier;
				rightVoltage *= multiplier;
			}

			this->tankSteerVoltage(leftVoltage, rightVoltage);
		}

		/**
		 * @brief Drive a drivetrain in skid steer mode
		 * 
		 * @param speed The desired speed of the drive
		 * @param turn The desired turning amount
		 */
		virtual void skidSteerVelocity(QSpeed speed, double turn) {}

		/**
		 * @brief Drive a drivetrain at using tank mode and velocity
		 * 
		 * @param leftSpeed The desired speed of the left drivetrain
		 * @param rightSpeed The desired speed of the right drivetrain
		 */
		virtual void tankSteerVelocity(QSpeed leftSpeed, QSpeed rightSpeed) {}

		/**
		 * @brief Drive a drivetrain using tank mode and voltage
		 * 
		 * @param leftVoltage The desired left voltage
		 * @param rightVoltage The desired right voltage
		 */
		virtual void tankSteerVoltage(double leftVoltage, double rightVoltage) {}

		QSpeed getMaxSpeed() {
			return maxSpeed;
		}

		~AbstractTankDrivetrain() {}
	};
} // namespace Pronounce

