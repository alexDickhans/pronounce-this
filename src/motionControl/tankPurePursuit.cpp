#include "tankPurePursuit.hpp"

namespace Pronounce {
	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain) : PurePursuit() {
		this->drivetrain = drivetrain;
	}

	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain, double lookaheadDistance) : PurePursuit(lookaheadDistance) {
		this->drivetrain = drivetrain;
	}

	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain, Odometry* odometry, double lookaheadDistance) : PurePursuit(odometry, lookaheadDistance) {
		this->drivetrain = drivetrain;
	}

	void TankPurePursuit::updateDrivetrain() {

		if (isDone(this->getStopDistance())) {
			return;
		}

		PurePursuitPointData pointData = this->getPointData();
		std::cout << "Curvature pointdata: " << this->getPointData().curvature << std::endl;

		pointData.localLookaheadVector.normalize();

		double dotProduct = pointData.localLookaheadVector.dot(Vector(1, 0));

		double speed = this->getSpeed() * dotProduct;
		
		bool inverted = signum_c(speed);

		// Drive backwards
		if (inverted) {
			pointData.curvature = -pointData.curvature;
			drivetrain->tankSteerVelocity(-speed * ((2 + pointData.curvature * this->drivetrain->getTrackWidth()) / 2), -speed * ((2 - pointData.curvature * this->drivetrain->getTrackWidth()) / 2));
		} else {
			drivetrain->tankSteerVelocity(speed * ((2 + pointData.curvature * this->drivetrain->getTrackWidth()) / 2), speed * ((2 - pointData.curvature * this->drivetrain->getTrackWidth()) / 2));
		}
	}

	void TankPurePursuit::stop() {
		drivetrain->tankSteerVelocity(0, 0);
	}

	TankPurePursuit::~TankPurePursuit() {
	}
} // namepsace Pronounce