#include "abstractTankDrivetrain.hpp"

namespace Pronounce {	
	AbstractTankDrivetrain::AbstractTankDrivetrain() {
	}

	AbstractTankDrivetrain::AbstractTankDrivetrain(double trackWidth) {
		this->trackWidth = trackWidth;
	}
	
	AbstractTankDrivetrain::~AbstractTankDrivetrain() {
	}
} // namespace Pronounce
