#include "motorButton.hpp"



namespace Pronounce
{
    MotorButton::MotorButton(pros::Controller* controller, pros::Motor* motor,
        pros::controller_digital_e_t positiveButton = pros::E_CONTROLLER_DIGITAL_L1,
        pros::controller_digital_e_t negativeButton = pros::E_CONTROLLER_DIGITAL_L2,
        int positiveAuthority = 0,
        int neutralAuthority = 0,
        int negativeAuthority = 0) {

        this->controller = controller;
        this->motor = motor;
        this->positiveButton = positiveButton;
        this->negativeButton = negativeButton;
        this->positiveAuthority = positiveAuthority;
        this->neutralAuthority = neutralAuthority;
        this->negativeAuthority = negativeAuthority;
    }

    void MotorButton::updateController() {
        if (!enabled)
            return;

        if (this->controller->get_digital(negativeButton)) {
            this->buttonStatus = NEGATIVE;
        }
        else if (this->controller->get_digital(positiveButton)) {
            this->buttonStatus = POSITIVE;
        }
        else {
            this->buttonStatus = NEUTRAL;
        }
    }

    void MotorButton::updateMotor() {
        if (!enabled)
            return;

        switch (this->buttonStatus) {
        case NEGATIVE:
            this->motor->move(negativeAuthority);
            break;
        case POSITIVE:
            this->motor->move(positiveAuthority);
            break;
        case NEUTRAL:
        default:
            this->motor->move(neutralAuthority);
            break;
        }
    }

    void MotorButton::update() {
        if (!enabled)
            return;
        this->updateController();
        this->updateMotor();
    }

    MotorButton::~MotorButton() {
    }
} // namespace Pronounce
