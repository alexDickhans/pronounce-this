#include "motorOdom.hpp"

namespace Pronounce {
    MotorOdom::MotorOdom(pros::Motor* motor, double radiusMM) {
        this->motor = motor;
        this->radius = radiusMM;
        this->motor->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    }

    MotorOdom::~MotorOdom() {
    }

    void MotorOdom::update() {
        this->setMM(motor->get_position() * radius * M_PI * 2.0);
    }
} // namespace Pronounce
