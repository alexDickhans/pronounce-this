#pragma once

#include "api.h"
#include "utils/utils.hpp"
#include "utils/motorGroup.hpp"
#include "abstractDrivetrain.hpp"

namespace Pronounce {
    /**
     * Abstract class as a structure for different drivetrains assuming there are 4 motors
     */
    class Drivetrain : public AbstractDrivetrain {
    protected:

    public:
		Drivetrain();
		
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

		~Drivetrain() {
			
		}
    };
} // namespace Pronounce

