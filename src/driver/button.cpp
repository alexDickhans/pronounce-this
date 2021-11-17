#include "button.hpp"

namespace Pronounce {
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
