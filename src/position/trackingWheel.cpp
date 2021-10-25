#include "trackingWheel.hpp"

namespace Pronounce {
    TrackingWheel::TrackingWheel(/* args */) {
    }

    TrackingWheel::TrackingWheel(pros::Rotation* rotationSensor) {
        this->rotationSensor = rotationSensor;
    }

    void TrackingWheel::update() {
        this->setPosition(rotationSensor->get_position() * this->getRadius() * M_PI * 2.0 * this->getTuningFactor());
    }

    TrackingWheel::~TrackingWheel() {
    }
} // namespace Pronounce
