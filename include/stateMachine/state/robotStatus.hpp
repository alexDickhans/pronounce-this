#pragma once

#include "stateMachine/behavior.hpp"
#include "api.h"

// TODO: Clean up
// TODO: Implement more sensors with classes

namespace Pronounce {

	enum GameMode {
		Skills = 0,
		Red = 1,
		Blue = 2,
	};

	const GameMode gameMode = GameMode::Skills; 

	double flywheelAdjustment = 0;
	double turretAngle = 0;

	class RobotStatus : public Behavior {
	private:
		double flywheelRPM = 2000;
		
	public:

		RobotStatus() {}

		void initialize() {
			// Init beambreaks
			return;
		}

		void update() {
			pros::vision_object_s_t biggestTurretDetection = turretVision.get_by_size(0);

			double angleChange = biggestTurretDetection.x_middle_coord;

			std::cout << "Vision sensor angle" << angleChange;

			turretAngle += map(angleChange, -320, 320, -0.01, 0.01);

			std::cout << "Vision sensor angle" << angleChange;
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
