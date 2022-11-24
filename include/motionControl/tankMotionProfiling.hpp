#pragma once

#include "velocityProfile/velocityProfile.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "stateMachine/behavior.hpp"
#include "time/robotTime.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"

namespace Pronounce {
	class TankMotionProfiling : public Behavior {
	private:
		pros::Mutex& drivetrainMutex;

		AbstractTankDrivetrain* drivetrain;
		VelocityProfile* velocityProfile;

		QTime startTime = 0.0;

		ContinuousOdometry* odometry;

		QCurvature curvature = 0.0;
	public:
		TankMotionProfiling(std::string name, AbstractTankDrivetrain* drivetrain, VelocityProfile* velocityProfile, ContinuousOdometry* odometry, pros::Mutex& drivetrainMutex) : Behavior(name), drivetrain(drivetrain), velocityProfile(velocityProfile), odometry(odometry), drivetrainMutex(drivetrainMutex) {

		}

		TankMotionProfiling(std::string name, AbstractTankDrivetrain* drivetrain, ProfileConstraints profileConstraints, QLength distance, ContinuousOdometry* odometry, pros::Mutex& drivetrainMutex, QCurvature curvature) : Behavior(name), drivetrain(drivetrain), odometry(odometry), drivetrainMutex(drivetrainMutex) {
			velocityProfile = new SinusoidalVelocityProfile(distance, profileConstraints);
			velocityProfile->calculate(100);
			this->curvature = curvature;
		}

		void initialize() {
			startTime = currentTime();
		}

		void update() {
			QTime duration = currentTime() - startTime;

			drivetrainMutex.take();

			std::cout << "InputDrivetrainSpeed: " << velocityProfile->getVelocityByTime(duration).Convert(inch/second) << std::endl;

			drivetrain->driveCurvature(velocityProfile->getVelocityByTime(duration), curvature);

			drivetrainMutex.give();
		}

		void exit() {
			drivetrainMutex.take();
			
			drivetrain->skidSteerVelocity(0.0, 0.0);

			drivetrainMutex.give();
		}

		bool isDone() {
			return currentTime() - startTime > velocityProfile->getDuration();
		}

		~TankMotionProfiling() {}
	};
} // namespace Pronounce
