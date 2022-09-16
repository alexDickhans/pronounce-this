#include "omniPurePursuit.hpp"

namespace Pronounce {
	OmniPurePursuit::OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain, ContinuousOdometry* odometry, PurePursuitProfile currentProfile) : PurePursuit(odometry, currentProfile) {
		this->drivetrain = drivetrain;
	}

	void OmniPurePursuit::updateDrivetrain(PurePursuitPointData pointData) {

		VelocityProfile velocityProfile = this->getCurrentProfile().velocityProfile;

		QSpeed lateralOutput = velocityProfile.getVelocityByDistance(pointData.distanceFromBeginning);

		// Get the turn target
		this->orientationPid->setTarget(this->getTurnTarget().getValue());

		// Get the turn output
		double turnOutput = this->orientationPid->update(this->getOdometry()->getPosition()->getAngle().getValue());

		// Send values to the drivetrain
		drivetrain->setDriveVectorVelocity(Vector(lateralOutput * 1_s, pointData.normalizedLookaheadVector.getAngle()), turnOutput);
	}

	void OmniPurePursuit::stop() {
		drivetrain->setDriveVectorVelocity(Vector(0.0, 0.0));
	}
	
	OmniPurePursuit::~OmniPurePursuit() {
	}
} // namespace Pronounce
