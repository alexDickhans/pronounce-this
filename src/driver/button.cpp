#include "button.hpp"

namespace Pronounce {
    Button::Button(pros::Controller* controller){
        this->controller = controller;
        this->positiveButton = pros::E_CONTROLLER_DIGITAL_L1;
        this->negativeButton = pros::E_CONTROLLER_DIGITAL_L2;
    }

    Button::Button(pros::Controller* controller, pros::controller_digital_e_t positiveButton, pros::controller_digital_e_t negativeButton) {
        this->controller = controller;
        this->positiveButton = positiveButton;
        this->negativeButton = negativeButton;
    }
    
    void Button::update() {
        updateController();
        updateActuator();
    }

    void Button::updateController() {
        if (controller->get_digital(positiveButton)) {
            buttonStatus = ButtonStatus::POSITIVE;
        } else if (controller->get_digital(negativeButton)) {
            buttonStatus = ButtonStatus::NEGATIVE;
        } else {
            buttonStatus = ButtonStatus::NEUTRAL;
        }
    }

    Button::~Button() {
    }
} // namespace Pronounce
