#include "simOdometry.hpp"

namespace Pronounce {
	SimOdometry::SimOdometry() {
		this->drivetrain = new SimDrivetrain();
		this->reset(new Position());
	}

	SimOdometry::SimOdometry(SimDrivetrain* drivetrain) {
		this->drivetrain = drivetrain;
		this->reset(new Position());
	}

	void SimOdometry::update() {
		Position position;
		position.operator=(this->getDrivetrain()->getPosition());
		position.operator+=(this->getResetPosition());
		this->getPosition()->operator=(this->getDrivetrain()->getPosition());
	}

	SimOdometry::~SimOdometry() {
	}
} // namespace Pronounce
