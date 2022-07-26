#pragma once

#include "chassis/abstractHolonomicDrivetrain.hpp"
#include "purePursuit.hpp"
#include "utils/vector.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"

namespace Pronounce {
	class OmniPurePursuit : public PurePursuit {
	private:
		AbstractHolonomicDrivetrain* drivetrain;
	public:
		OmniPurePursuit();
		OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain);
		OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain, double lookaheadDistance);
		OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain, ContinuousOdometry* odometry, double lookaheadDistance);
		
		void updateDrivetrain();

		void stop();

		AbstractHolonomicDrivetrain* getDrivetrain() {
			return drivetrain;
		}

		void setDrivetrain(AbstractHolonomicDrivetrain* drivetrain) {
			this->drivetrain = drivetrain;
		}

		~OmniPurePursuit();
	};
} // namespace Pronounce
