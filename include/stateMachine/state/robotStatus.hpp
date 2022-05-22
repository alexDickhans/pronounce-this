#pragma once

#include "behavior.hpp"

namespace Pronounce {
	class RobotStatus : public Behavior {
	private:
		/* data */
	public:
		RobotStatus(/* args */);

		void initialize() {
			return;
		}

		void update() {

		}

		void exit() {

		}

		~RobotStatus();
	};
} // namespace Pronounce
