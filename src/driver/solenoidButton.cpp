#include "solenoidButton.hpp"

namespace Pronounce {
    SolenoidButton::SolenoidButton(pros::Controller* controller) : Button(controller) {
    }

    SolenoidButton::SolenoidButton(pros::Controller* controller, pros::ADIDigitalOut* solenoid) : Button(controller) {
        this->solenoid = solenoid;
    }

	SolenoidButton::SolenoidButton(pros::Controller* controller, pros::controller_digital_e_t positiveButton) : Button(controller, positiveButton) {
        this->solenoid = new pros::ADIDigitalOut(9);
    }

    SolenoidButton::SolenoidButton(pros::Controller* controller, pros::controller_digital_e_t positiveButton, pros::controller_digital_e_t negativeButton) : Button(controller, positiveButton, negativeButton) {
        this->solenoid = new pros::ADIDigitalOut(9);
    }

    void SolenoidButton::updateActuator() {
        if (this->getSingleToggle()) {
            this->solenoid->set_value(this->getButtonStatus());
        }

        switch (this->getButtonStatus()) {
        case ButtonStatus::POSITIVE:
            this->solenoid->set_value(!this->inverted);
            break;
        case ButtonStatus::NEGATIVE:
            this->solenoid->set_value(this->inverted);
            break;
        case ButtonStatus::NEUTRAL:
        default:
            this->solenoid->set_value(this->inverted);
            break;
        }
    }
} // Pronounce