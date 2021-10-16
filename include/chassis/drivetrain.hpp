#pragma once

#include "api.h"
#include "utils/utils.hpp"

namespace Pronounce {
    /**
     * Abstract class as a structure for different drivetrains assuming there are 4 motors
     */
    class Drivetrain {
    protected:
        pros::Motor* frontLeft;
        pros::Motor* frontRight;
        pros::Motor* backLeft;
        pros::Motor* backRight;

        pros::Imu* imu;

    public:

        Drivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);

        /**
         * Get average temperature of all the motors.
         */
        double getTemp();

        /**
         * Get average speed of all the motors
         */
        double getSpeed();

        

        /**
         * Get frontLeft motor
         * 
         * @return frontLeft motor pointer
         */
        pros::Motor* getFrontLeft() {
            return this->frontLeft;
        }

        /**
         * Set frontLeft motor
         * 
         * @param frontLeft Motor pointer
         */
        void setFrontLeft(pros::Motor* frontLeft) {
            this->frontLeft = frontLeft;
        }
        
        /**
         * Get frontRight motor
         * 
         * @return frontRight motor pointer
         */
        pros::Motor* getFrontRight() {
            return this->frontRight;
        }

        /**
         * Set frontRight motor
         * 
         * @param frontRight Motor pointer
         */
        void setFrontRight(pros::Motor* frontRight) {
            this->frontRight = frontRight;
        }

        /**
         * Get backLeft motor
         * 
         * @return backLeft motor pointer
         */
        pros::Motor* getBackLeft() {
            return this->frontLeft;
        }

        /**
         * Set backLeft motor
         * 
         * @param backLeft Motor pointer
         */
        void setBackLeft(pros::Motor* backLeft) {
            this->backLeft = backLeft;
        }

        /**
         * Get backRight motor
         * 
         * @return backRight motor pointer
         */
        pros::Motor* getBackRight() {
            return this->frontLeft;
        }

        /**
         * Set backRight motor
         * 
         * @param backRight Motor pointer
         */
        void setBackRight(pros::Motor* backRight) {
            this->backRight = backRight;
        }

    };
} // namespace Pronounce

