#pragma once

#include "api.h"
#include "purePursuit.hpp"
#include "chassis/tankDrive.hpp"
#include <algorithm>

namespace Pronounce {
	class TankPurePursuit : public PurePursuit {
	private:
		TankDrivetrain* drivetrain;

		double speed = 100;
		bool inverted = false;
	public:
		TankPurePursuit(TankDrivetrain* drivetrain);
		TankPurePursuit(TankDrivetrain* drivetrain, double lookaheadDistance);
		TankPurePursuit(TankDrivetrain* drivetrain, Odometry* odometry, double lookaheadDistance);

		void updateDrivetrain();

		void stop();

		TankDrivetrain* getDrivetrain() {
			return drivetrain;
		}

		void setDrivetrain(TankDrivetrain* drivetrain) {
			this->drivetrain = drivetrain;
		}

		double getSpeed() {
			return speed;
		}

		void setSpeed(double speed) {
			this->speed = speed;
		}

		bool getInverted() {
			return inverted;
		}

		void setInverted(bool inverted) {
			this->inverted = inverted;
		}

		~TankPurePursuit();
	};
} // namespace Pronounce
