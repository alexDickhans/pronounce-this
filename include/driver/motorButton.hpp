#pragma once

#include "api.h"
#include "controller.hpp"

namespace Pronounce {

    typedef enum {
        NEUTRAL = 0,
        POSITIVE = 1,
        NEGATIVE = 2
    } ButtonStatus;

    class MotorButton {
    private:
        ButtonStatus buttonStatus;
        bool enabled;
        int positiveAuthority = 0;
        int neutralAuthority = 0;
        int negativeAuthority = 0;

        pros::Motor* motor;
        pros::Controller* controller;
        pros::controller_digital_e_t positiveButton, negativeButton;

        void updateController();
        void updateMotor();

    public:
        MotorButton();
        MotorButton(pros::Controller* controller, pros::Motor* motor,
            pros::controller_digital_e_t positiveButton,
            pros::controller_digital_e_t negativeButton,
            int positiveAuthority,
            int neutralAuthority,
            int negativeAuthority);

        ButtonStatus getButtonStatus() {
            return buttonStatus;
        }

        void update();

        ~MotorButton();
    };

} // namespace Pronounce

