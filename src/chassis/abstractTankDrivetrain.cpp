#include "abstractTankDrivetrain.hpp"

namespace Pronounce {	
	AbstractTankDrivetrain::AbstractTankDrivetrain() : AbstractTankDrivetrain(0.0) {
	}

	AbstractTankDrivetrain::AbstractTankDrivetrain(double trackWidth) : AbstractDrivetrain() {
		this->trackWidth = trackWidth;
	}
	
	AbstractTankDrivetrain::~AbstractTankDrivetrain() {
	}
} // namespace Pronounce
