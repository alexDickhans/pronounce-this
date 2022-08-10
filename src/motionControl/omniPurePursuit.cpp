#include "omniPurePursuit.hpp"

namespace Pronounce {
	OmniPurePursuit::OmniPurePursuit() : PurePursuit() {
		drivetrain = new AbstractHolonomicDrivetrain();
	}

	OmniPurePursuit::OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain) : PurePursuit() {
		this->drivetrain = drivetrain;
	}

	OmniPurePursuit::OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain, QLength lookaheadDistance) : PurePursuit(lookaheadDistance) {
		this->drivetrain = drivetrain;
	}

	OmniPurePursuit::OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain, ContinuousOdometry* odometry, QLength lookaheadDistance) : PurePursuit(odometry, lookaheadDistance) {
		this->drivetrain = drivetrain;
	}

	void OmniPurePursuit::updateDrivetrain(PurePursuitPointData pointData) {

		QSpeed lastSpeed = this->drivetrain->getSpeed();

		QSpeed maxSpeed = sqrt(2.0 * this->getCurrentProfile().maxAcceleration.getValue() * pointData.distanceFromEnd.getValue() + ConvertTo(2.0_in, m));
		maxSpeed = maxSpeed > 5.0_inchs ? maxSpeed : 5_inchs;

		QSpeed maxAccelerationFrame = this->getCurrentProfile().maxAcceleration * this->getUpdateTime();
		QSpeed speed = 0.0;
		if (maxSpeed > speed) {
			speed = clamp(this->getCurrentProfile().speed, this->drivetrain->getSpeed() - maxAccelerationFrame, this->drivetrain->getSpeed() + maxAccelerationFrame);
		}
		else {
			speed = this->getCurrentProfile().speed;
		}
		
		QSpeed motorSpeed = clamp(clamp(speed.getValue(), -maxSpeed.getValue(), maxSpeed.getValue()), -this->getCurrentProfile().speed.getValue(), this->getCurrentProfile().speed.getValue());

		QSpeed lateralOutput = motorSpeed;

		// Get the turn target
		this->getCurrentProfile().orientationPid->setTarget(this->getTurnTarget().getValue());

		// Get the turn output
		double turnOutput = this->getCurrentProfile().orientationPid->update(this->getOdometry()->getPosition()->getAngle().getValue());

		// Send values to the drivetrain
		drivetrain->setDriveVectorVelocity(Vector(lateralOutput, pointData.normalizedLookaheadVector.getAngle()), turnOutput);
	}

	void OmniPurePursuit::stop() {
		drivetrain->setDriveVectorVelocity(Vector(0.0, 0.0));
	}
	
	OmniPurePursuit::~OmniPurePursuit() {
	}
} // namespace Pronounce
