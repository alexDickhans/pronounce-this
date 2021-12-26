#pragma once

#include "api.h"
#include "purePursuit.hpp"
#include "chassis/tankDrive.hpp"

namespace Pronounce {
	class TankPurePursuit : public PurePursuit {
	private:
		TankDrivetrain* drivetrain;
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

		~TankPurePursuit();
	};
} // namespace Pronounce
