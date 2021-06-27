#include "motorOdom.hpp"

namespace Pronounce {
    MotorOdom::MotorOdom(pros::Motor* motor, double radiusMM) {
        this->motor = motor;
        this->radius = radiusMM;
    }
    
    MotorOdom::~MotorOdom() {
    }

    void MotorOdom::update() {
        this->setMM(motor->get_position() * radius * M_PI * 2);
    }
} // namespace Pronounce
