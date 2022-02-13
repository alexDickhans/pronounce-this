#include "simDrivetrain.hpp"

namespace Pronounce {

	SimDrivetrain::SimDrivetrain() {
		this->position = new Position();
	}

	SimDrivetrain::SimDrivetrain(Position* position) {
		this->position = position;
	}
	
	SimDrivetrain::~SimDrivetrain() {
	}
} // namespace Pronounce
