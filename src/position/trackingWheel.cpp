#include "trackingWheel.hpp"

namespace Pronounce {
    TrackingWheel::TrackingWheel(/* args */) {
    }

    TrackingWheel::TrackingWheel(pros::Rotation* rotationSensor) {
        this->rotationSensor = rotationSensor;
    }

    TrackingWheel::~TrackingWheel() {
    }
} // namespace Pronounce
