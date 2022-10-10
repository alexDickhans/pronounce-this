#pragma once

#include "stateMachine/behavior.hpp"
#include "api.h"
#include "hardware/hardware.hpp"

// TODO: Clean up
// TODO: Implement more sensors with classes

namespace Pronounce {

	enum GameMode {
		Skills = 0,
		Red = 1,
		Blue = 2,
	};

	const GameMode gameMode = GameMode::Red;

	class RobotStatus : public Behavior {
	private:
	public:

		RobotStatus() {}

		void initialize() {
			return;
		}

		void update() {
		}

		void exit() {
			return;
		}
		
		~RobotStatus() {}
	};
} // namespace Pronounce
