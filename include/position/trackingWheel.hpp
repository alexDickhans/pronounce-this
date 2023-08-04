#pragma once

#include "odomWheel.hpp"
#include "api.h"

// TODO: rename to be more descriptive
// TODO: Add docstrings
// TODO: add comments

namespace Pronounce
{
    class TrackingWheel : public OdomWheel {
    private:
        std::shared_ptr<pros::Rotation> rotationSensor;
    public:
        TrackingWheel();
        TrackingWheel(std::shared_ptr<pros::Rotation> rotationSensor, QLength radius);

        void reset() {
            rotationSensor->reset_position();
            this->setPosition(0.0);
            this->setLastPosition(0.0);
        }

        void update() {
            this->setPosition((rotationSensor->get_position() / 36000.0) * this->getRadius().getValue() * ((double) 2_pi * this->getTuningFactor()));
        }

        ~TrackingWheel();
    };
} // namespace Pronounce


