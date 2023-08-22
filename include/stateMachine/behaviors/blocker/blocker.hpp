#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
    class Blocker : public Behavior {
    private:
        pros::Motor& winch;
        double setpoint{0.0};
    public:
        Blocker(std::string name, pros::Motor& winch, double setpoint);

        void initialize() {
            winch
        }

        ~Blocker();
    };
    
    Blocker::Blocker(std::string name, pros::Motor& winch, double setpoint) : Behavior(name), winch(winch) {
        this->setpoint = setpoint;
    }
    
    Blocker::~Blocker() {
    }
    
} // namespace Pronounce
