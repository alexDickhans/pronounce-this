#pragma once

#include "api.h"
#include "utils/utils.hpp"
#include "utils/motorGroup.hpp"

namespace Pronounce {
    /**
     * Abstract class as a structure for different drivetrains assuming there are 4 motors
     */
    class Drivetrain {
    protected:

		MotorGroup leftMotors;
		MotorGroup rightMotors;

        pros::Imu* imu;

    public:
		Drivetrain();
        Drivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);
		Drivetrain(MotorGroup leftMotors, MotorGroup rightMotors, pros::Imu* imu);

        /**
         * Get average temperature of all the motors.
         */
        double getTemp();

        /**
         * Get average speed of all the motors
         */
        double getSpeed();

		pros::Imu* getImu() {
			return imu;
		}

		void setImu(pros::Imu* imu) {
			this->imu = imu;
		}

		MotorGroup getLeftMotors() {
			return leftMotors;
		}

		void setLeftMotors(MotorGroup leftMotors) {
			this->leftMotors = leftMotors;
		}

		void addLeftMotor(pros::Motor* motor) {
			this->leftMotors.addMotor(motor);
		}

		MotorGroup getRightMotors() {
			return rightMotors;
		}

		void setRightMotors(MotorGroup rightMotors) {
			this->rightMotors = rightMotors;
		}

		void addRightMotor(pros::Motor* motor) {
			this->rightMotors.addMotor(motor);
		}

        virtual void update() {}
    };
} // namespace Pronounce

