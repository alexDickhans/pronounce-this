#pragma once

#include "api.h"
#include "controller.hpp"
#include "button.hpp"

namespace Pronounce {

    class MotorButton : public Button {
    private:
        bool goToImmediately = false;

        int positiveAuthority = 0;
        int neutralAuthority = 0;
        int negativeAuthority = 0;

        int min = 0;
        int max = 0;
        pros::Motor* motor;

    public:
        MotorButton(pros::Controller* controller, pros::Motor* motor,
        pros::controller_digital_e_t positiveButton = pros::E_CONTROLLER_DIGITAL_L1,
        pros::controller_digital_e_t negativeButton = pros::E_CONTROLLER_DIGITAL_L2,
        int positiveAuthority = 0,
        int neutralAuthority = 0,
        int negativeAuthority = 0,
        int min = 0,
        int max = 0);
        MotorButton();
        
        void updateActuator();

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

