#include "tankDrive.hpp"

namespace Pronounce {

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
		MotorOdom* leftPivot = new MotorOdom(frontLeft, 2);
		MotorOdom* rightPivot = new MotorOdom(frontRight, 2);
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

		bool reversed = this->targetPosition->getTheta() < 0;

		double xDiff = this->targetPosition->getX() - currentPosition->getX();
		double yDiff = this->targetPosition->getY() - currentPosition->getY();

		double distance = sqrt(pow(xDiff, 2) + pow(yDiff, 2)) * (reversed ? -1 : 1);
		double linearPosition = sqrt(pow(this->targetPosition->getX() - this->startingPosition->getX(), 2) + pow(this->targetPosition->getY() - this->startingPosition->getY(), 2)) - distance;
		double angle = nullRotationDistance < distance ? atan2(yDiff, xDiff) + (reversed ? 180 : 0) : prevAngle;
		this->prevAngle = angle;

		this->movePid->setPosition(linearPosition);
		this->movePid->setTarget(distance);
		this->turnPid->setPosition(imu->get_heading());
		this->turnPid->setTarget(this->getStopped() ? this->angle : toDegrees(angle));

		double lateral = this->movePid->update();
		double turn = this->turnPid->update();

		this->getFrontLeft()->move(std::clamp(lateral + turn, -maxVoltage, maxVoltage));
		this->getFrontLeft()->move(std::clamp(lateral - turn, -maxVoltage, maxVoltage));
	}

	bool TankDrivetrain::getStopped() {
		// Derivitive ~= speed 
		return this->movePid->getDerivitive() < speedThreshhold && this->movePid->getError() < errorThreshhold ||
			this->turnPid->getDerivitive() < turnThreshhold && this->turnPid->getError() < turnErrorThreshhold;
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
