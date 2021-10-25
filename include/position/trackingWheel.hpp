#pragma once

#include "odomWheel.hpp"
#include "api.h"

namespace Pronounce
{
    class TrackingWheel : public OdomWheel {
    private:
        pros::Rotation* rotationSensor;
    public:
        TrackingWheel();
        TrackingWheel(pros::Rotation* rotationSensor);

        void update() {
            this->setPosition(rotationSensor->get_position() * this->getRadius() * M_PI * 2.0 * this->getTuningFactor());
        }

        ~TrackingWheel();
    };
} // namespace Pronounce


