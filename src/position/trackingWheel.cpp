#include "trackingWheel.hpp"

namespace Pronounce {
    TrackingWheel::TrackingWheel(/* args */) {
    }

    TrackingWheel::TrackingWheel(std::shared_ptr<pros::Rotation> rotationSensor, QLength radius) : OdomWheel(radius) {
        this->rotationSensor = rotationSensor;
    }

    TrackingWheel::~TrackingWheel() {
    }
} // namespace Pronounce
