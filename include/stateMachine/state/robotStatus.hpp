#pragma once

#include "behavior.hpp"
#include "api.h"

namespace Pronounce {

	pros::Vision turretVision(18, VISION_ZERO_CENTER);

	class RobotStatus : public Behavior {
	private:

	public:
		RobotStatus(/* args */);

		void initialize() {
			// Init beambreaks
			return;
		}

		void update() {

		}

		void exit() {
			return;
		}

		~RobotStatus();
	};
} // namespace Pronounce
