#include "motorOdom.hpp"

namespace Pronounce {
    MotorOdom::MotorOdom(pros::Motor* motor, double radius) {
        this->motor = motor;
        this->radius = radius;
        this->motor->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    }

    MotorOdom::~MotorOdom() {
    }

    void MotorOdom::update() {
        this->setPosition(motor->get_position() * radius * M_PI * 2.0 * tuningFactor);
    }
} // namespace Pronounce
