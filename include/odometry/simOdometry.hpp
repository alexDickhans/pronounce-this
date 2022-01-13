#pragma once

#include "chassis/simDrivetrain.hpp"
#include "odometry.hpp"

namespace Pronounce {
	class SimOdometry : public Odometry {
	private:
		SimDrivetrain* drivetrain;
	public:
		SimOdometry(); 
		SimOdometry(SimDrivetrain* drivetrain);

		void update();

		void reset(Position* position) {
			this->drivetrain->setPosition(position);
            this->getPosition()->operator=(position);
            this->getResetPosition()->operator=(position);
        }

		SimDrivetrain* getDrivetrain() {
			return drivetrain;
		}

		void setDrivetrain(SimDrivetrain* drivetrain) {
			this->drivetrain = drivetrain;
		}

		~SimOdometry();
	};	
} // namespace Pronounce
