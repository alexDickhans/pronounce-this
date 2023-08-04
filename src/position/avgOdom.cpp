#include "avgOdom.hpp"

namespace Pronounce {
    AvgOdom::AvgOdom() {

	}

	AvgOdom::AvgOdom(std::vector<OdomWheel> odomWheels) {
		this->odomWheels = odomWheels;
	}

    AvgOdom::~AvgOdom() {
    }
} // namespace Pronounce
