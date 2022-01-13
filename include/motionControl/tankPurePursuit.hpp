#pragma once

#include "purePursuit.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include <iostream>

namespace Pronounce {
	class TankPurePursuit : public PurePursuit {
	private:
		AbstractTankDrivetrain* drivetrain;

		double speed = 100;
		bool inverted = false;
	public:
		TankPurePursuit(AbstractTankDrivetrain* drivetrain);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, double lookaheadDistance);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, Odometry* odometry, double lookaheadDistance);

		void updateDrivetrain();

		void stop();

		AbstractTankDrivetrain* getDrivetrain() {
			return drivetrain;
		}

		void setDrivetrain(AbstractTankDrivetrain* drivetrain) {
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
