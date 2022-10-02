#pragma once

#include "stateMachine/behavior.hpp"
#include "stateMachine/state/flywheelController.hpp"
#include "api.h"

// TODO: Clean up
// TODO: Implement more sensors with classes

namespace Pronounce {

	enum GameMode {
		Skills = 0,
		Red = 1,
		Blue = 2,
	};

	// const GameMode gameMode = GameMode::Red;

	double flywheelAdjustment = 0;
	Angle turretAdjustment = 90_deg;
	double currentFlywheelRPM = 2300;

	class RobotStatus : public Behavior {
	private:
		Angle turretAngle = 0.0;

		FlywheelController allianceGoal;
		FlywheelController opponentGoal;
	public:

		RobotStatus() : allianceGoal(ALLIANCE_GOAL), opponentGoal(OPPONENT_GOAL) {}

		void initialize() {
			// Init beambreaks
			return;
		}

		void update() {
			/// pros::vision_object_s_t biggestTurretDetection = turretVision.get_by_size(0);

			// double angleChange = biggestTurretDetection.x_middle_coord;
			std::cout << "OutputDrivetrainSpeed: " << drivetrain.getSpeed().getValue() << std::endl;

			FlywheelValue flywheelValues = this->allianceGoal.getFlywheelValue(*odometry.getPosition(), odometry.getCurrentVelocity());

			if (pros::competition::is_autonomous()) {
				// this->turretAngle = flywheelValues.turretAngle - odometry.getAngle();
				// currentFlywheelRPM = flywheelValues.flywheelSpeed;
			}
		}

		void exit() {
			return;
		}

		double getFlywheelTarget() {
			return currentFlywheelRPM + flywheelAdjustment;
		}

		double getActualFlywheelRpm() {
			return launcherIdle.getFlywheelSpeed();
		}

		void setFlywheelRPM(double flywheelRPM1) {
			currentFlywheelRPM = flywheelRPM1;
		}

		Angle getTurretAngle() {
			return angleDifference((turretAngle + turretAdjustment).Convert(radian), 0.0);
		}

		~RobotStatus() {}
	};
} // namespace Pronounce
