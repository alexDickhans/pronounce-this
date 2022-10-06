#pragma once

#include "velocityProfile/velocityProfile.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "stateMachine/behavior.hpp"
#include "api.h"
#include "odometry/continuousOdometry/continuousOdometry.hpp"

namespace Pronounce {
	class TankMotionProfiling : public Behavior {
	private:
		pros::Mutex& drivetrainMutex;

		AbstractTankDrivetrain& drivetrain;
		VelocityProfile velocityProfile;

		ContinuousOdometry& odometry;
	public:
		TankMotionProfiling(std::string name, AbstractTankDrivetrain& drivetrain, VelocityProfile velocityProfile, ContinuousOdometry& odometry, pros::Mutex& drivetrainMutex) : Behavior(name), drivetrain(drivetrain), velocityProfile(velocityProfile), odometry(odometry), drivetrainMutex(drivetrainMutex) {

		}

		TankMotionProfiling(std::string name, AbstractTankDrivetrain& drivetrain, ProfileConstraints profileConstraints, QLength distance, ContinuousOdometry& odometry, pros::Mutex& drivetrainMutex) : Behavior(name), drivetrain(drivetrain), odometry(odometry), drivetrainMutex(drivetrainMutex) {
			velocityProfile = SinusoidalVelocityProfile(distance, profileConstraints);
		}

		~TankMotionProfiling() {}
	};
} // namespace Pronounce
