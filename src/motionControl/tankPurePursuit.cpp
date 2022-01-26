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

		std::cout << "Orientation control: " << this->orientationControl << std::endl;

		if (orientationControl) {
			double currentOrientation = this->getOdometry()->getPosition()->getTheta();
			this->turnPid->setPosition(angleDifference(currentOrientation, 0));
			double spinSpeed = this->turnPid->update();

			std::cout << "Angle difference: " << this->turnPid->getError() << std::endl;

			this->drivetrain->skidSteerVelocity(0, spinSpeed * speed);

			printf("Spin speed: %f\n", spinSpeed);

			return;
		}

		if (isDone(this->getStopDistance())) {
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
		double scalar = 1; // pointData.lookaheadVector.getMagnitude() / this->getLookahead(); 

		double speed = clamp(this->getSpeed() * side * scalar, -this->getSpeed(), this->getSpeed());
		
		std::cout << "Lookahead vector: " << pointData.localLookaheadVector.getCartesian().to_string() << std::endl;
		std::cout << "Lookahead position: " << pointData.lookaheadPoint.to_string() << std::endl;
		std::cout << "Lookahead angle: " << pointData.localLookaheadVector.getAngle() << std::endl;

		drivetrain->driveCurvature(speed, pointData.curvature);
	}

	void TankPurePursuit::stop() {
		drivetrain->tankSteerVelocity(0, 0);
	}

	TankPurePursuit::~TankPurePursuit() {
	}
} // namepsace Pronounce