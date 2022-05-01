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

			this->drivetrain->skidSteerVelocity(0, spinSpeed * speed * this->getOutputMultiplier());

			return;
		}
		else if (isDone(this->getStopDistance())) {
			this->stop();
			return;
		}

		PurePursuitPointData pointData = this->getPointData();

		double side = sqrt(abs(clamp(pointData.localLookaheadVector.getCartesian().getY() / this->getLookahead(), -1.0, 1.0))) * signnum_c(pointData.localLookaheadVector.getCartesian().getY());

		side = abs(side) < 0.5 ? signnum_c(side) : side;

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

		double maxSpeed = sqrt(2 * this->getMaxAcceleration() * pointData.distanceFromEnd + 2.0);
		maxSpeed = maxSpeed > 5 ? maxSpeed : 5;

		double updateTime = pros::millis() - lastUpdateTime;
		lastUpdateTime = updateTime;
		double maxAccelerationFrame = this->getMaxAcceleration() * updateTime / 1000;
		double speed = 0;
		if (maxSpeed > speed) {
			speed = clamp(this->getSpeed() * side, this->drivetrain->getSpeed() - maxAccelerationFrame, this->drivetrain->getSpeed() + maxAccelerationFrame);
		}
		else {
			speed = this->getSpeed() * side;
		}

		// printf("Max speed: %f\n", maxSpeed);
		// printf("Side: %f\n", side);
		// printf("Local Lookahead x: %f, y: %f\n", pointData.localLookaheadVector.getCartesian().getX(), pointData.localLookaheadVector.getCartesian().getY());
		// printf("x: %f, y: %f\n", currentPoint.getX(), currentPoint.getY());
		// printf("Lookahead Point: %f, %f\n", pointData.lookaheadPoint.getX(), pointData.lookaheadPoint.getY());
		// printf(std::string("Path: " + currentPath.getName()).c_str());
		// printf("\n\n");

		double motorSpeed = clamp(clamp(speed, -maxSpeed, maxSpeed), -this->getSpeed(), this->getSpeed()) * this->getOutputMultiplier();

		if (useVoltage) {
			drivetrain->driveCurvatureVoltage(motorSpeed * (12000/600.0), pointData.curvature);
		} else {
			drivetrain->driveCurvature(motorSpeed, pointData.curvature);
		}
	}

	void TankPurePursuit::stop() {
		drivetrain->tankSteerVelocity(0, 0);
	}

	TankPurePursuit::~TankPurePursuit() {
	}
} // namepsace Pronounce