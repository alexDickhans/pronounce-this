#include "tankPurePursuit.hpp"

namespace Pronounce {
	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain) : PurePursuit() {
		this->drivetrain = drivetrain;
		this->turnPid = new PID();
		this->turnPid->setTurnPid(true);
	}

	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain, double lookaheadDistance) : PurePursuit(lookaheadDistance) {
		this->drivetrain = drivetrain;
		this->turnPid = new PID();
		this->turnPid->setTurnPid(true);
	}

	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain, Odometry* odometry, double lookaheadDistance) : PurePursuit(odometry, lookaheadDistance) {
		this->drivetrain = drivetrain;
		this->turnPid = new PID();
		this->turnPid->setTurnPid(true);
	}

	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain, Odometry* odometry, PID* turnPid, double lookaheadDistance) : PurePursuit(odometry, lookaheadDistance) {
		this->drivetrain = drivetrain;
		this->turnPid = turnPid;
		this->turnPid->setTurnPid(true);
	}

	void TankPurePursuit::updateDrivetrain() {

		if (!this->isEnabled()) {
			return;
		}

		if (orientationControl) {
			double currentOrientation = this->getOdometry()->getPosition()->getTheta();
			this->turnPid->setPosition(angleDifference(currentOrientation, 0));
			double spinSpeed = this->turnPid->update();

			std::cout << "Angle difference: " << this->turnPid->getError() << std::endl;

			this->drivetrain->skidSteerVelocity(0, spinSpeed * speed);

			return;
		}
		else if (isDone(this->getStopDistance())) {
			this->stop();
			return;
		}

		PurePursuitPointData pointData = this->getPointData();

		double side = sqrt(abs(clamp(pointData.localLookaheadVector.getCartesian().getY() / this->getLookahead(), -1.0, 1.0))) * signum_c(pointData.localLookaheadVector.getCartesian().getY());

		side = abs(side) < 0.5 ? signum_c(side) : side;

		// Redundant scalar, could be used in the future
		// Found that using the Y values got the same affect when this was wanted and a better affect when it wasn't
		// Using the same equation with Y instead.
		// Time difference on test path(In sim)
		// Before change: 3.4667
		// After change: 3.08
		// 12% improvement, mostly in speed up and slow down.
		double lastSpeed = this->drivetrain->getSpeed();

		Path currentPath = this->getPath(this->getCurrentPathIndex());
		Point currentPoint = Point(this->getOdometry()->getPosition()->getX(), this->getOdometry()->getPosition()->getY());

		double accelTime = 200 / this->getMaxAcceleration();
		double accelDistance = 0.5 * this->getMaxAcceleration() * accelTime * accelTime;
		double multiplier = 200 / sqrt(accelDistance / 3.33);
		double maxSpeed = multiplier * sqrt(this->getPath(this->getCurrentPathIndex()).distanceFromEnd(Point(this->getOdometry()->getPosition()->getX(), this->getOdometry()->getPosition()->getY())) + 2.0);
		maxSpeed = maxSpeed > 20 ? maxSpeed : 20;

		double updateTime = pros::millis() - lastUpdateTime;
		lastUpdateTime = updateTime;
		double maxAccelerationFrame = this->getMaxAcceleration() * updateTime / 1000;
		double speed = 0;
		if (maxSpeed > speed) {
			speed = clamp(this->getSpeed() * side, this->drivetrain->getSpeed() - maxAccelerationFrame, this->drivetrain->getSpeed() + maxAccelerationFrame);
		} else {
			speed = this->getSpeed() * side;
		}

		printf("Max speed: %f\n", maxSpeed);
		printf("Multiplier: %f\n", multiplier);
		printf("AccelDistance: %f\n", accelDistance);

		double motorSpeed = clamp(clamp(speed, -maxSpeed, maxSpeed), -this->getSpeed(), this->getSpeed());

		drivetrain->driveCurvature(motorSpeed, pointData.curvature);
	}

	void TankPurePursuit::stop() {
		drivetrain->tankSteerVelocity(0, 0);
	}

	TankPurePursuit::~TankPurePursuit() {
	}
} // namepsace Pronounce