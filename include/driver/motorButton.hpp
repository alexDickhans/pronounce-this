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
        bool enabled = true;
        bool goToImmediately = false;
        int positiveAuthority = 0;
        int neutralAuthority = 0;
        int negativeAuthority = 0;

        int min = 0;
        int max = 0;

        pros::Motor* motor;
        pros::Controller* controller;
        pros::controller_digital_e_t positiveButton, negativeButton;

        void updateController();
        void updateMotor();

    public:
        MotorButton(pros::Controller* controller, pros::Motor* motor,
            pros::controller_digital_e_t positiveButton,
            pros::controller_digital_e_t negativeButton,
            int positiveAuthority,
            int neutralAuthority,
            int negativeAuthority,
            int min,
            int max);
        MotorButton();

        ButtonStatus getButtonStatus() {
            return buttonStatus;
        }

        bool getEnabled() {
            return this->enabled;
        }

        void setEnabled(bool enabled) {
            this->enabled = enabled;
        }

        bool getGoToImmediately() {
            return this->goToImmediately;
        }

        void setGoToImmediately(bool goToImmediately) {
            this->goToImmediately = goToImmediately;
        }

        void update();

        ~MotorButton();
    };

} // namespace Pronounce

