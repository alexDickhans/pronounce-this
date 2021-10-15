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

	void TankDrivetrain::update() {

		if (!enabled)
			return;

		tankOdom->update();

		Position* currentPosition = tankOdom->getPosition();

		double xDiff = this->targetPosition->getX() - currentPosition->getX();
		double yDiff = this->targetPosition->getY() - currentPosition->getY();

		double angle = atan(yDiff / xDiff);
		double distance = sqrt(pow(xDiff, 2) + pow(yDiff, 2));

		this->movePid->setTarget(distance);
		this->turnPid->setTarget(toDegrees(angle));

		double lateral = this->movePid->update();
		double turn = nullRotationDistance < distance ? 0.0 : this->turnPid->update();

		this->getFrontLeft()->move(lateral + turn);
		this->getFrontLeft()->move(lateral - turn);
	}

	TankDrivetrain::~TankDrivetrain() {
	}
} // namespace Pronounce
