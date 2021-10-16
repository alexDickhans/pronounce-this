#include "tankDrive.hpp"

namespace Pronounce {

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
		MotorOdom* leftPivot = new MotorOdom(frontLeft, 50.8);
		MotorOdom* rightPivot = new MotorOdom(frontRight, 50.8);
		this->tankOdom = new TankOdom(leftPivot, rightPivot, imu);

		this->targetPosition = new Position();
		this->startingPosition = new Position();

		this->turnPid = new PID(0, 0, 0, 0, 0);
		this->movePid = new PID(0, 0, 0, 0, 0);
	}

	void TankDrivetrain::reset() {
		this->tankOdom->setPosition(startingPosition);
		this->tankOdom->reset();
	}

	void TankDrivetrain::update() {

		if (!enabled)
			return;

		tankOdom->update();

		Position* currentPosition = tankOdom->getPosition();

		double xDiff = this->targetPosition->getX() - currentPosition->getX();
		double yDiff = this->targetPosition->getY() - currentPosition->getY();

		double angle = atan2(yDiff, xDiff);
		double distance = sqrt(pow(xDiff, 2) + pow(yDiff, 2));

		this->movePid->setTarget(distance);
		this->turnPid->setTarget(this->getStopped() ? this->angle : toDegrees(angle));

		double lateral = this->movePid->update();
		double turn = nullRotationDistance < distance ? 0.0 : this->turnPid->update();

		this->getFrontLeft()->move(std::clamp(lateral + turn, -maxVoltage, maxVoltage));
		this->getFrontLeft()->move(std::clamp(lateral - turn, -maxVoltage, maxVoltage));
	}

	bool TankDrivetrain::getStopped() {
		// Derivitive ~= speed 
		return this->movePid->getDerivitive() < speedThreshhold && this->movePid->getError() < errorThreshhold;
	}

	void TankDrivetrain::waitForStop() {
		if (!enabled)
			return;

		while (!this->getStopped()) {
			pros::Task::delay(20);
		}
	}

	TankDrivetrain::~TankDrivetrain() {
	}
} // namespace Pronounce
