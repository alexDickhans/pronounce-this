#pragma once

#include <utility>

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
    bool hasTriball = false;
    bool gotTriball = false;

    class Intake : public Behavior {
    private:
        pros::AbstractMotor& intake;
        double intakeSpeed{0};
    public:
        Intake(std::string name, pros::AbstractMotor& intake, double intakeSpeed) : Behavior(std::move(name)), intake(intake), intakeSpeed(intakeSpeed) {

        }

        void initialize() override {
            intake.move_voltage(intakeSpeed * 12000.0);
        }

        void update() override {

        }

        void exit() override {
            intake.move_voltage(0);
        }

        bool isDone() override {
            return false;
        }

        ~Intake() {}
    };
} // namespace Pronounce
