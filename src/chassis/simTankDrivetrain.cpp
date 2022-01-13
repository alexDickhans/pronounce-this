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
		double leftChange = clamp(leftVelocityTarget - leftVelocity, -maxAcceleration, maxAcceleration);
		double rightChange = clamp(rightVelocityTarget - rightVelocity, -maxAcceleration, maxAcceleration);

		leftVelocity = clamp(leftVelocity + leftChange, -maxSpeed, maxSpeed);
		rightVelocity = clamp(rightVelocity + rightChange, -maxSpeed, maxSpeed);

		// Calculate the local offset
		double offset = (leftVelocity + rightVelocity) / 2.0;
		double angle = (rightVelocity - leftVelocity) / this->getTrackWidth();

		leftDistance += leftVelocity;
		rightDistance += rightVelocity;

		double relativeAngle = ((leftDistance - rightDistance) / this->getTrackWidth()) + this->getResetOrientation();

		// Calculate a vector
		Vector localOffset(offset, relativeAngle+M_PI_2);

		Position* newPosition = new Position();
		newPosition->operator=(oldPosition);
		
		newPosition->add(localOffset.getCartesian());
		newPosition->setTheta(relativeAngle - (angle / 2.0));

		this->setPosition(newPosition);
	}
	
	SimTankDrivetrain::~SimTankDrivetrain() {
	}
} // namespace Pronounce
