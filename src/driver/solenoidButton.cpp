#include "solenoidButton.hpp"

namespace Pronounce {
    SolenoidButton::SolenoidButton(pros::Controller* controller) : Button(controller) {
        this->solenoid = new pros::ADIDigitalOut(9);
    }
    
    SolenoidButton::SolenoidButton(pros::Controller* controller, pros::ADIDigitalOut* solenoid) : Button(controller) {
        this->solenoid = solenoid;
    }

    SolenoidButton::SolenoidButton(pros::Controller* controller, pros::controller_digital_e_t positiveButton, pros::controller_digital_e_t negativeButton) : Button(controller, positiveButton, negativeButton) {
        this->solenoid = new pros::ADIDigitalOut(9);
    }

    void SolenoidButton::updateActuator() {
        switch (this->getButtonStatus()) {
            case ButtonStatus::POSITIVE:
                this->solenoid->set_value(!this->inverted);
                break;
            case ButtonStatus::NEGATIVE:
                this->solenoid->set_value(this->inverted);
                break;
            case ButtonStatus::NEUTRAL:
            default:
                if (retainOnNeutral) {
                    // Nothing
                } else {
                    this->solenoid->set_value(this->inverted);
                }
                break;
        }
    }
} // Pronounce