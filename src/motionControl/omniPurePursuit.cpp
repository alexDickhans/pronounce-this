#include "omniPurePursuit.hpp"

namespace Pronounce {
	OmniPurePursuit::OmniPurePursuit() : PurePursuit() {
		drivetrain = new AbstractHolonomicDrivetrain();
	}

	OmniPurePursuit::OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain) : PurePursuit() {
		this->drivetrain = drivetrain;
	}

	OmniPurePursuit::OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain, double lookaheadDistance) : PurePursuit(lookaheadDistance) {
		this->drivetrain = drivetrain;
	}

	OmniPurePursuit::OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain, Odometry* odometry, double lookaheadDistance) : PurePursuit(odometry, lookaheadDistance) {
		this->drivetrain = drivetrain;
	}

	void OmniPurePursuit::updateDrivetrain() {

		PurePursuitPointData pointData = this->getPointData();

		double lastSpeed = this->drivetrain->getSpeed();

		double maxSpeed = sqrt(2 * this->getCurrentProfile().maxAcceleration * pointData.distanceFromEnd + 2.0);
		maxSpeed = maxSpeed > 5 ? maxSpeed : 5;

		double maxAccelerationFrame = this->getCurrentProfile().maxAcceleration * this->getUpdateTime() / 1000;
		double speed = 0;
		if (maxSpeed > speed) {
			speed = clamp(this->getCurrentProfile().speed, this->drivetrain->getSpeed() - maxAccelerationFrame, this->drivetrain->getSpeed() + maxAccelerationFrame);
		}
		else {
			speed = this->getCurrentProfile().speed;
		}
		
		double motorSpeed = clamp(clamp(speed, -maxSpeed, maxSpeed), -this->getCurrentProfile().speed, this->getCurrentProfile().speed) * this->getOutputMultiplier();

		double lateralOutput = motorSpeed;

		// Get the turn target
		this->getCurrentProfile().orientationPid->setTarget(this->getTurnTarget());

		// Get the turn output
		double turnOutput = this->getCurrentProfile().orientationPid->update(this->getOdometry()->getPosition()->getTheta());

		// Send values to the drivetrain
		drivetrain->setDriveVectorVelocity(Vector(lateralOutput, pointData.normalizedLookaheadVector.getAngle()), turnOutput);
	}

	void OmniPurePursuit::stop() {
		drivetrain->setDriveVectorVelocity(Vector(0.0, 0.0));
	}
	
	OmniPurePursuit::~OmniPurePursuit() {
	}
} // namespace Pronounce
