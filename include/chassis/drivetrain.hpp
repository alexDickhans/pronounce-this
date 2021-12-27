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

		MotorGroup getLeftMotors() {
			return leftMotors;
		}

		void setLeftMotors(MotorGroup leftMotors) {
			this->leftMotors = leftMotors;
		}

		MotorGroup getRightMotors() {
			return rightMotors;
		}

		void setRightMotors(MotorGroup rightMotors) {
			this->rightMotors = rightMotors;
		}

        virtual void update() {}
    };
} // namespace Pronounce

