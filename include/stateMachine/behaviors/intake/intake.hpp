#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
    bool hasTriball = false;
    bool gotTriball = false;

    class Intake : public Behavior {
    private:
        pros::Motor_Group& intake;
        double intakeSpeed{0};

        bool exitWithTriball{false};

        bool stalled{false};
        uint32_t startTime = 0;
        const double thresholdCurrent = 750;
        const uint32_t thresholdTime = 250;
    public:
        Intake(std::string name, pros::Motor_Group& intake, double intakeSpeed, bool exit) : Behavior(name), intake(intake), intakeSpeed(intakeSpeed), exitWithTriball(exit) {

        }

        void initialize() {
            intake.move_voltage(intakeSpeed * 12000);
            startTime = pros::millis();
            
            if (intakeSpeed < -0.05) {
                hasTriball = false;
            }
        }

        void update() {
            // Leave voltage where it is once the program starts
            double totalCurrent = 0;

            for (double current : this->intake.get_current_draws()) {
                totalCurrent += current;
            }

            double averageCurrent = totalCurrent/this->intake.size();

            if (averageCurrent < thresholdCurrent) {
                startTime = pros::millis();
            }            

            stalled = (pros::millis() - startTime) > thresholdTime;

            gotTriball = false;

            if (stalled && exitWithTriball) {
                hasTriball = true;
                gotTriball = true;
            }
        }

        void exit() {
            intake.move_voltage(0);
        }

        bool isDone() {
            return false; //xitWithTriball && stalled;
        }

        ~Intake() {}
    };
} // namespace Pronounce