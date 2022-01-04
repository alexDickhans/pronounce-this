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

		Position* oldPosition = this->getPosition();
		
		// Update the wheel velocities
		double leftChange = std::clamp(leftVelocityTarget - leftVelocity, -maxAcceleration, maxAcceleration);
		double rightChange = std::clamp(rightVelocityTarget - rightVelocity, -maxAcceleration, maxAcceleration);

		leftVelocity = std::clamp(leftVelocity + leftChange, -maxSpeed, maxSpeed);
		rightVelocity = std::clamp(rightVelocity + rightChange, -maxSpeed, maxSpeed);

		// Calculate the local offset
		double offset = (leftVelocity + rightVelocity) / 2.0;
		double angle = (leftVelocity - rightVelocity) / this->getTrackWidth();

		leftDistance += leftVelocity;
		rightDistance += rightVelocity;

		double relativeAngle = (leftDistance - rightDistance) / this->getTrackWidth();

		// Calculate a vector
		Vector localOffset(offset, relativeAngle);

		Position* newPosition = new Position();
		newPosition->operator=(oldPosition);
		
		newPosition->add(localOffset.getCartesian());
		newPosition->setTheta(relativeAngle);

		this->setPosition(newPosition);
	}
	
	SimTankDrivetrain::~SimTankDrivetrain() {
	}
} // namespace Pronounce
