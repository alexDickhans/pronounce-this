#include "button.hpp"

namespace Pronounce {
    Button::Button(pros::Controller* controller) {
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
        // If in autonomous mode don't update the controller
        if (autonomous)
            return;

        // Update the controller if the controller toggles with a button
        if (singleToggle) {
            // Toggle button status between neutral and positive
            if (controller->get_digital_new_press(positiveButton)) {
                printf("Button pressed\n");
                buttonStatus = buttonStatus == ButtonStatus::POSITIVE ? ButtonStatus::NEUTRAL : ButtonStatus::POSITIVE;
                printf("Button status: %d\n", buttonStatus);
            } else if (controller->get_digital_new_press(negativeButton)) {
                printf("Button pressed\n");
                buttonStatus = buttonStatus == ButtonStatus::NEGATIVE ? ButtonStatus::NEUTRAL : ButtonStatus::NEGATIVE;
                printf("Button status: %d\n", buttonStatus);
            }
            return;
        }

        // Update button status
        if (controller->get_digital(positiveButton)) {
            buttonStatus = ButtonStatus::POSITIVE;
        }
        else if (controller->get_digital(negativeButton)) {
            buttonStatus = ButtonStatus::NEGATIVE;
        } else if (retainOnNeutral) {
            // Don't do anything
        } else {
            buttonStatus = ButtonStatus::NEUTRAL;
        }
    }

    Button::~Button() {
    }
} // namespace Pronounce
