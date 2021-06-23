#pragma once

#include "api.h"
#include "drivetrain.hpp"

#define AUTON_CONTROL 0
#define DRIVER_CONTROL 1

class XDrive {
private:

    // Motors, have to be pointers because they would 
    // not be modifiable otherwise 
    pros::Motor* frontRight;
    pros::Motor* frontLeft;
    pros::Motor* backRight;
    pros::Motor* backLeft;

    // Motion control wheels

    pros::Imu* imu;

    int currentState;

public:

    XDrive(pros::Motor* frontRight,
        pros::Motor* frontLeft,
        pros::Motor* backRight,
        pros::Motor* backLeft,
        pros::Imu* imu) {

        this->frontRight = frontRight;
        this->frontLeft = frontLeft;
        this->backRight = backRight;
        this->backLeft = backLeft;

        this->imu = imu;
    } //! Move implementation to src/chassis/xdrive.cpp


    /**
     * Get the average drivetrain temperature
     * 
     * @return Average of the 4 wheels
     */
    double getAvgTemp() {
        double total = this->frontLeft->get_temperature() +
            this->frontRight->get_temperature() +
            this->backLeft->get_temperature() +
            this->backRight->get_temperature();
        return total / 4;
    } //! Move implementation to src/chassis/xdrive.cpp

};