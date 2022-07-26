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

        void reset() {
            rotationSensor->reset_position();
            this->setPosition(0.0);
            this->setLastPosition(0.0);
        }

        void update() {
            this->setPosition(((rotationSensor->get_position() / 100.0)/360.0) * this->getRadius().getValue() * 1_pi * 2.0 * this->getTuningFactor());
        }

        ~TrackingWheel();
    };
} // namespace Pronounce


