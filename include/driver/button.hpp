#pragma once

#include "api.h"

namespace Pronounce {

    typedef enum {
        NEUTRAL = 0,
        POSITIVE = 1,
        NEGATIVE = 2
    } ButtonStatus;

    class Button {
    private:
        bool enabled = true;

        ButtonStatus buttonStatus;
        pros::Controller* controller;
        pros::controller_digital_e_t positiveButton, negativeButton;
    public:
        Button(pros::Controller* cotroller);
        Button(pros::Controller* cotroller, pros::controller_digital_e_t positiveButton, pros::controller_digital_e_t negativeButton);

        void update();
        void updateController();
        virtual void updateActuator() {}

        ButtonStatus getButtonStatus() {
            return buttonStatus;
        }

        bool getEnabled() {
            return this->enabled;
        }

        void setEnabled(bool enabled) {
            this->enabled = enabled;
        }
        ~Button();
    };

    Button::Button(Controller) {
    }
} // namespace Pronounce
