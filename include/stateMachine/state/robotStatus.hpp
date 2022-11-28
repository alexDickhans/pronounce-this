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
		int differentFrameCount = 0;
		int discCount;
		bool inIntakePath;
	public:

		RobotStatus() {}

		void initialize() {
			return;
		}

		void update() {
			if (inIntakePath ? intakeLineSensor.get_value() > 2000 : intakeLineSensor.get_value() < 2000) {
				differentFrameCount++;
			}
			else {
				differentFrameCount = 0;
			}

			if (differentFrameCount == 10) {
				inIntakePath = !inIntakePath;
				if (!inIntakePath) {
					discCount++;
					std::cout << "DiscCount: " << discCount << std::endl;
					if (discCount == 3) {
						ptoStateController.setCurrentBehavior(&ptoIntakeStopped);
					}
				}
			}

			if (ptoStateController.getCurrentBehavior() == &ptoCatapult) {
				discCount = 0;
				std::cout << "DiscCount: " << discCount << std::endl;
			}

			std::cout << "IntakeLineSensor: " << intakeLineSensor.get_value() << std::endl;
		}

		void exit() {
			return;
		}

		void setDiscCount(int discCount) {
			this->discCount = discCount;
		} 

		~RobotStatus() {}
	};
} // namespace Pronounce
