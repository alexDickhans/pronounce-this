#pragma once

#include "stateMachine/behavior.hpp"
#include "api.h"
#include "hardware/hardware.hpp"
#include "auton.h"

// TODO: Clean up
// TODO: Implement more sensors with classes

namespace Pronounce {

	class RobotStatus: public Behavior {
	private:
		int differentFrameCount = 0;
		bool inIntakePath;
		int discCountFrames;
	public:
		int discCount;

		RobotStatus() {}

		void initialize() {
			return;
		}

		void update() {
			if (inIntakePath ? intakeLineSensor.get_value() > 2600 : intakeLineSensor.get_value() < 2000) {
				differentFrameCount++;
			}
			else {
				differentFrameCount = 0;
			}

			if (differentFrameCount == 2) {
				inIntakePath = !inIntakePath;
				if (!inIntakePath) {
					discCount++;
					std::cout << "DiscCount: " << discCount << std::endl;
				}
			}

			if (discCount >= 3) {
				discCountFrames ++;
			} else {
				discCountFrames = 0;
			}

			if (discCountFrames == 30) {
				ptoStateController.setCurrentBehavior(&ptoIntakeStopped);
			}

			if (catapultLimitSwitch.get_angle() < 20000) {
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
