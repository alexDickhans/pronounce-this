#pragma once

#include "chassis/omniDrivetrain.hpp"
#include "purePursuit.hpp"
#include "utils/vector.hpp"
#include "odometry/odometry.hpp"

namespace Pronounce {
	class OmniPurePursuit : public PurePursuit {
	private:
		OmniDrivetrain* drivetrain;
	public:
		OmniPurePursuit();
		OmniPurePursuit(OmniDrivetrain* drivetrain);
		OmniPurePursuit(OmniDrivetrain* drivetrain, double lookaheadDistance);
		OmniPurePursuit(OmniDrivetrain* drivetrain, Odometry* odometry, double lookaheadDistance);
		
		void updateDrivetrain();

		void stop();

		OmniDrivetrain* getDrivetrain() {
			return drivetrain;
		}

		void setDrivetrain(OmniDrivetrain* drivetrain) {
			this->drivetrain = drivetrain;
		}

		~OmniPurePursuit();
	};
} // namespace Pronounce
