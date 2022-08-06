#pragma once

#include "stateMachine/behavior.hpp"
#include "api.h"

// TODO: Clean up
// TODO: Implement more sensors with classes

namespace Pronounce {

	pros::Vision turretVision(18, VISION_ZERO_CENTER);

	double flywheelAdjustment = 0;
	double turretAngle = 0;

	class RobotStatus : public Behavior {
	private:
		double flywheelRPM = 3500;
		
	public:

		RobotStatus() {}

		void initialize() {
			// Init beambreaks
			return;
		}

		void update() {
		}

		void exit() {
			return;
		}

		double getFlywheelTarget() {
			return flywheelRPM + flywheelAdjustment;
		}

		double getActualFlywheelRpm() {
			return launcherIdle.getFlywheelSpeed();
		}

		double getTurretAngle() {
			return turretAngle;
		}

		~RobotStatus() {}
	};
} // namespace Pronounce
