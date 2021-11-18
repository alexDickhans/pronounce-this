#pragma once

#include "api.h"
#include "button.hpp"

namespace Pronounce {
    class SolenoidButton : public Button {
    private:
        pros::ADIDigitalOut solenoid;

        bool inverted{false};
        bool retainOnNeutral{true};
    public:
        SolenoidButton(pros::Controller* controller);
        SolenoidButton(pros::Controller* controller, pros::ADIDigitalOut* solenoid);
        SolenoidButton(pros::Controller* controller, pros::controller_digital_e_t positiveButton, pros::controller_digital_e_t negativeButton);

        void updateActuator();

        bool getInverted() {
            return inverted;
        }

        void setInverted(bool inverted) {
            this->inverted = inverted;
        }

        pros::ADIDigitalOut getSolenoid() {
            return solenoid;
        }

        void setSolenoid(pros::ADIDigitalOut solenoid) {
            this->solenoid = solenoid;
        }
    }
} // Pronounce
