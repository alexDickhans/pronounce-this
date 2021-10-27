#include "tankDrive.hpp"

std::shared_ptr<okapi::Logger> visionLogger = std::make_shared<okapi::Logger>(
	okapi::TimeUtilFactory::createDefault().getTimer(),
	"/ser/sout", // "/usd/visionLog.txt",
	okapi::Logger::LogLevel::debug // Show everything
	);

namespace Pronounce {

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
		MotorOdom* leftPivot = new MotorOdom(frontLeft, 2);
		MotorOdom* rightPivot = new MotorOdom(frontRight, 2);
		AvgOdom* odomWheels = new AvgOdom(new std::list<OdomWheel>(leftPivot, rightPivot));
		this->tankOdom = new TankOdom(odomWheels, imu);

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

		if (!enabled) {
			return;
		}

		tankOdom->update();

		Position* currentPosition = tankOdom->getPosition();

		bool reversed = this->targetPosition->getTheta() < 0;

		double xDiff = this->targetPosition->getX() - currentPosition->getX();
		double yDiff = this->targetPosition->getY() - currentPosition->getY();

		double angleToTarget = toDegrees(atan2(yDiff, xDiff));
		double robotAngleToTarget = fmod(angleToTarget - imu->get_heading() + 360, 360.0);
		bool flipDistance = -90 < fmod(robotAngleToTarget,  360.0) < 90;// Allow the robot to reverse when it passes the target

		double distance = sqrt(pow(xDiff, 2) + pow(yDiff, 2)) * (flipDistance ? -1 : 1) * (reversed ? -1 : 1);
		double linearPosition = sqrt(pow(this->targetPosition->getX() - this->startingPosition->getX(), 2) + pow(this->targetPosition->getY() - this->startingPosition->getY(), 2)) - distance;
		double angle = nullRotationDistance < distance ? angleToTarget + (reversed ? 180 : 0) : prevAngle;
		this->prevAngle = angle;

		this->movePid->setPosition(linearPosition);
		this->movePid->setTarget(isnan(this->angle) ? distance : linearPosition);
		this->turnPid->setPosition(imu->get_rotation());
		this->turnPid->setTarget(isnan(this->angle) ? angle : this->angle);

		double lateral = std::clamp(this->movePid->update(), -127.0, 127.0);
		double turn = this->turnPid->update();

		visionLogger.get()->debug<std::string>("imu angle: " + std::to_string(turnPid->getPosition())
								 + " robotAngleToTarget:" + std::to_string(robotAngleToTarget) 
								 + " X:" + std::to_string(currentPosition->getX())
								 + " Y:" + std::to_string(currentPosition->getY())
								 + " TargetX:" + std::to_string(targetPosition->getX())
								 + " targetY:" + std::to_string(targetPosition->getY())
								 + " heading:" + std::to_string(imu->get_heading()));// std::clamp(lateral, -maxVoltage, maxVoltage)));

		this->getFrontLeft()->move(std::clamp(lateral + turn, -maxVoltage, maxVoltage));
		this->getBackLeft()->move(std::clamp(lateral + turn, -maxVoltage, maxVoltage));
		this->getFrontRight()->move(std::clamp(lateral - turn, -maxVoltage, maxVoltage));
		this->getBackRight()->move(std::clamp(lateral - turn, -maxVoltage, maxVoltage));
	}

	bool TankDrivetrain::getStopped() {
		// Derivitive ~= speed 
		return abs(this->movePid->getError()) < errorThreshhold &&
			abs(this->turnPid->getError()) < turnErrorThreshhold;
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
