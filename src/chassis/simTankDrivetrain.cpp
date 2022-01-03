#include "simTankDrivetrain.hpp"

namespace Pronounce {
	
	SimTankDrivetrain::SimTankDrivetrain() : SimTankDrivetrain(0.0, 0.0, 0.0) {}

	SimTankDrivetrain::SimTankDrivetrain(double trackWidth) : SimTankDrivetrain(trackWidth, 0.0, 0.0) {}

	SimTankDrivetrain::SimTankDrivetrain(double trackWidth, double maxAcceleration, double maxSpeed) : SimTankDrivetrain(trackWidth, maxAcceleration, maxSpeed, new Position(0.0, 0.0, 0.0)) {}

	SimTankDrivetrain::SimTankDrivetrain(double trackWidth, double maxAcceleration, double maxSpeed, Position* position) : SimDrivetrain(position), AbstractTankDrivetrain(trackWidth) {
		this->maxAcceleration = maxAcceleration;
		this->maxSpeed = maxSpeed;
	}

	void SimTankDrivetrain::update() {
		SimDrivetrain::update();
		
		// Update the wheel velocities
		double leftChange = std::clamp(leftVelocityTarget - leftVelocity, -maxAcceleration, maxAcceleration);
		double rightChange = std::clamp(rightVelocityTarget - rightVelocity, -maxAcceleration, maxAcceleration);

		leftVelocity = std::clamp(leftVelocity + leftChange, -maxSpeed, maxSpeed);
		rightVelocity = std::clamp(rightVelocity + rightChange, -maxSpeed, maxSpeed);

		// Calculate the local offset
		double offset = (leftVelocity + rightVelocity) / 2;
		double angle = (rightVelocity - leftVelocity) / this->getTrackWidth();

		// Calculate a vector
		Vector localOffset(offset, angle);

		// Update the position
		this->getPosition()->add(localOffset.getCartesian());

		this->getPosition()->setTheta(this->getPosition()->getTheta() + angle);
	}
	
	SimTankDrivetrain::~SimTankDrivetrain() {
	}
} // namespace Pronounce
