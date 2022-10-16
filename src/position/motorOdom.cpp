#include "motorOdom.hpp"

namespace Pronounce {
    MotorOdom::MotorOdom(std::shared_ptr<pros::Motor> motor, QLength radius) : OdomWheel(radius), motor(motor) {
        this->motor->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    }

    MotorOdom::~MotorOdom() {
    }

    void MotorOdom::update() {
        this->setPosition(motor->get_position() * this->getRadius().Convert(metre) * 1_pi * 2.0 * this->getTuningFactor());
    }
} // namespace Pronounce
