#include "avgOdom.hpp"

namespace Pronounce {
    AvgOdom::AvgOdom(std::list<OdomWheel> odomWheels) {
        this->odomWheels = odomWheels;
    }

    AvgOdom::AvgOdom(std::list<OdomWheel>* odomWheels) {
        this->odomWheels = *odomWheels;
    }

    AvgOdom::~AvgOdom() {
    }
} // namespace Pronounce
