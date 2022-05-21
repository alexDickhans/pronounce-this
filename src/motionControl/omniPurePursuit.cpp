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
		// Get the lookahead vector from the pointData
		Vector normalizedLookaheadVector = this->getPointData().normalizedLookaheadVector;

		// Set the lateralPid values
		this->getCurrentProfile().getLateralPid()->setTarget(normalizedLookaheadVector.getMagnitude());
		this->getCurrentProfile().getLateralPid()->setPosition(0);

		// Get the lateralPid output
		double lateralOutput = this->getCurrentProfile().getLateralPid()->update();

		// Get the turn target
		this->getCurrentProfile().getTurnPid()->setTarget(this->getTurnTarget());
		this->getCurrentProfile().getTurnPid()->setPosition(this->getOdometry()->getPosition()->getTheta());

		// Get the turn output
		double turnOutput = this->getCurrentProfile().getTurnPid()->update();

		// Send values to the drivetrain
		drivetrain->setDriveVectorVelocity(Vector(lateralOutput, normalizedLookaheadVector.getAngle()), turnOutput);
	}

	void OmniPurePursuit::stop() {
		drivetrain->setDriveVectorVelocity(Vector(0.0, 0.0));
	}
	
	OmniPurePursuit::~OmniPurePursuit() {
	}
} // namespace Pronounce
