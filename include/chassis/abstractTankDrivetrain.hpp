#pragma once

#include "utils/utils.hpp"
#include "units/units.hpp"
#include <iostream>

namespace Pronounce {
	typedef struct TankChassisSpeeds_ {
		QVelocity speed = 0.0;
		QCurvature curvature = 0.0;
	} TankChassisSpeeds;

	/**
	 * @brief Abstract tank drivetrain class for all tank drivetrains
	 * 
	 * @authors Alex Dickhans(ad101-lab)
	 */
	class AbstractTankDrivetrain {
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
		QVelocity maxSpeed;
	public:
		/**
		 * @brief Construct a new Abstract Tank Drivetrain object with all values set to zero. 
		 * Doing any functions that require the track width will require the value set
		 * 
		 */
		AbstractTankDrivetrain() : trackWidth(0.0), maxSpeed(0.0) {}

		/**
		 * @brief Construct a new Abstract Tank Drivetrain object with trackWidth set
		 * 
		 * @param trackWidth The trackWidth of the drivetrain
		 */
		AbstractTankDrivetrain(QLength trackWidth, QVelocity maxSpeed) : trackWidth(trackWidth), maxSpeed(maxSpeed) {}

		/**
		 * @brief Get the current speed of the robot
		 *
		 * @return QVelocity The current speed of the robot
		 */
		virtual QVelocity getVelocity() { return 0.0; }

		/**
		 * @brief Get the current speed of the robot
		 * 
		 * @return QVelocity The current speed of the robot
		 */
		[[deprecated("Use getVelocityInstead")]] virtual QVelocity getSpeed() { return getVelocity(); }

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
		 * @brief Drive a drivetrain using tank mode and voltage
		 * 
		 * @param leftVoltage The desired left voltage
		 * @param rightVoltage The desired right voltage
		 */
		virtual void tankSteerVoltage(int32_t leftVoltage, int32_t rightVoltage) {}

		virtual void reset() {}

		virtual QLength getDistanceSinceReset() { return 0.0; }

		QVelocity getMaxSpeed() {
			return maxSpeed;
		}

		~AbstractTankDrivetrain() {}
	};
} // namespace Pronounce

