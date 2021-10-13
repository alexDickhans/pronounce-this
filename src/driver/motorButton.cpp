#include "motorButton.hpp"



namespace Pronounce
{
    MotorButton::MotorButton(pros::Controller* controller, pros::Motor* motor,
        pros::controller_digital_e_t positiveButton = pros::E_CONTROLLER_DIGITAL_L1,
        pros::controller_digital_e_t negativeButton = pros::E_CONTROLLER_DIGITAL_L2,
        int positiveAuthority = 0,
        int neutralAuthority = 0,
        int negativeAuthority = 0,
        int min = 0,
        int max = 0) {

        this->controller = controller;
        this->motor = motor;
        this->positiveButton = positiveButton;
        this->negativeButton = negativeButton;
        this->positiveAuthority = positiveAuthority;
        this->neutralAuthority = neutralAuthority;
        this->negativeAuthority = negativeAuthority;
        this->min = min;
        this->max = max;
    }

    void MotorButton::updateController() {

        if (this->controller->get_digital(negativeButton)) {
            this->buttonStatus = NEGATIVE;
        }
        else if (this->controller->get_digital(positiveButton)) {
            this->buttonStatus = POSITIVE;
        }
        else if (!goToImmediately) {
            this->buttonStatus = NEUTRAL;
        }
    }

    void MotorButton::updateMotor() {
        if (!enabled) {
            this->motor->move(0.0);
            return;
        }

        switch (this->buttonStatus) {
        case NEGATIVE:
            if (goToImmediately) {
                this->motor->move_absolute(min, negativeAuthority);
            }
            else if (this->motor->get_position() > this->min && this->min < this->max) {
                this->motor->move(negativeAuthority);
            } else if (this->min >= this->max) {
                this->motor->move(negativeAuthority);
            }
            else {
                this->motor->move(neutralAuthority);
            }
            break;
        case POSITIVE:
            if (goToImmediately) {
                this->motor->move_absolute(max, positiveAuthority);
            }
            else if (this->motor->get_position() < this->max && this->min < this->max) {
                this->motor->move(positiveAuthority);
            }else if (this->min >= this->max) {
                this->motor->move(positiveAuthority);
            }
            else {
                this->motor->move(neutralAuthority);
            }
            break;
        case NEUTRAL:
        default:
            if (goToImmediately)
                return;
            this->motor->move(neutralAuthority);
            break;
        }


    }

    void MotorButton::update() {
        this->updateController();
        this->updateMotor();
    }

    MotorButton::~MotorButton() {
    }
} // namespace Pronounce
