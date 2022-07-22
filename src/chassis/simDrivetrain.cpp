#include "simDrivetrain.hpp"

namespace Pronounce {

	SimDrivetrain::SimDrivetrain() {
		this->position = new Pose2D();
	}

	SimDrivetrain::SimDrivetrain(Pose2D* position) {
		this->position = position;
	}
	
	SimDrivetrain::~SimDrivetrain() {
	}
} // namespace Pronounce
