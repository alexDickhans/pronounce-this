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

	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain, ContinuousOdometry* odometry, double lookaheadDistance) : PurePursuit(odometry, lookaheadDistance) {
		this->drivetrain = drivetrain;
		this->turnPid = new PID();
		this->turnPid->setTurnPid(true);
	}

	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain, ContinuousOdometry* odometry, PID* turnPid, double lookaheadDistance) : PurePursuit(odometry, lookaheadDistance) {
		this->drivetrain = drivetrain;
		this->turnPid = turnPid;
		this->turnPid->setTurnPid(true);
	}

	void TankPurePursuit::updateDrivetrain() {

		if (orientationControl) {
			Angle currentOrientation = this->getOdometry()->getPosition()->getAngle();
			double spinSpeed = this->turnPid->update(angleDifference(currentOrientation.getValue(), 0));

			std::cout << "Angle difference: " << this->turnPid->getError() << std::endl;

			this->drivetrain->skidSteerVelocity(0, spinSpeed * speed.getValue() * this->getOutputMultiplier());

			return;
		}
		else if (isDone(this->getStopDistance())) {
			this->stop();
			return;
		}

		PurePursuitPointData pointData = this->getPointData();

		double side = sqrt(abs(clamp(pointData.localLookaheadVector.getCartesian().getY().getValue() / this->getCurrentProfile().lookaheadDistance.getValue(), -1.0, 1.0))) * signnum_c(pointData.localLookaheadVector.getCartesian().getY().getValue());

		side = abs(side) < 0.5 ? signnum_c(side) : side;

		// Redundant scalar, could be used in the future
		// Found that using the Y values got the same affect when this was wanted and a better affect when it wasn't
		// Using the same equation with Y instead.
		// Time difference on test path(In sim)
		// Before change: 3.4667
		// After change: 3.08
		// 12% improvement, mostly in speed up and slow down.
		QSpeed lastSpeed = this->drivetrain->getSpeed();

		Path currentPath = this->getPath();
		Point currentPoint = Point(this->getOdometry()->getPosition()->getX(), this->getOdometry()->getPosition()->getY());

		QSpeed maxSpeed = sqrt(2 * this->getCurrentProfile().maxAcceleration.getValue() * pointData.distanceFromEnd.getValue() + 2.0);
		maxSpeed = maxSpeed > 5.0_inchs ? maxSpeed : 5.0_inchs;

		QSpeed maxAccelerationFrame = this->getCurrentProfile().maxAcceleration * this->getUpdateTime();
		QSpeed speed = 0.0;
		if (maxSpeed > speed) {
			speed = clamp(this->getSpeed().getValue() * side, (this->drivetrain->getSpeed() - maxAccelerationFrame).getValue(), (this->drivetrain->getSpeed() + maxAccelerationFrame).getValue());
		}
		else {
			speed = this->getSpeed().getValue() * side;
		}

		double motorSpeed = clamp(clamp(speed.getValue(), -maxSpeed.getValue(), maxSpeed.getValue()), -this->getSpeed().getValue(), this->getSpeed().getValue()) * this->getOutputMultiplier();

		if (useVoltage) {
			drivetrain->driveCurvatureVoltage(this->getSpeed().getValue() * this->getOutputMultiplier() * side * (12000/600.0), pointData.curvature);
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