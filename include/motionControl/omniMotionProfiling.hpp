#pragma once

#include "velocityProfile/velocityProfile.hpp"
#include "chassis/abstractHolonomicDrivetrain.hpp"
#include "stateMachine/behavior.hpp"
#include "units/units.hpp"
#include "api.h"
#include "utils/vector.hpp"
#include "time/robotTime.hpp"
#include "stateMachine/behaviors/drivetrain/initDrivetrain.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"

namespace Pronounce {
	class OmniMotionProfiling : public Behavior {
	private:
		VelocityProfile& velocityProfile;
		AbstractHolonomicDrivetrain& drivetrain;
		Angle direction;
		Angle targetAngle;

		QTime startTime = 0.0;

		double speedMultiplier = 200.0/34.0;

		PID* pid;
		ContinuousOdometry* odometry;

	public:
		OmniMotionProfiling(std::string name, VelocityProfile& velocityProfile, AbstractHolonomicDrivetrain& drivetrain, PID* pid, ContinuousOdometry* odometry, Angle direction, Angle targetAngle);

		void initialize() {
			startTime = currentTime();
			pid->reset();
			pid->setTarget(targetAngle.Convert(radian));
		}

		void update() {
			QTime duration = currentTime() - startTime;

			double outputPower = pid->update(odometry->getPose()->getAngle().Convert(radian));

			drivetrain.setDriveVectorVelocity(Vector((velocityProfile.getVelocityByTime(duration) * 1_s).Convert(inch) * speedMultiplier, direction), outputPower);
		}

		void exit() {
			drivetrain.setDriveVectorVelocity(Vector(0.0, 0.0));
		}

		bool isDone() {
			return currentTime() - startTime > velocityProfile.getDuration();
		}

		void setVelocityProfile(VelocityProfile& velocityProfile) {
			this->velocityProfile = velocityProfile;
		}

		~OmniMotionProfiling();
	};
	
	OmniMotionProfiling::OmniMotionProfiling(std::string name, VelocityProfile& velocityProfile, AbstractHolonomicDrivetrain& drivetrain, PID* pid, ContinuousOdometry* odometry, Angle direction, Angle targetAngle) : velocityProfile(velocityProfile), drivetrain(drivetrain), direction(direction), pid(pid), targetAngle(targetAngle), odometry(odometry), Behavior(name) {
		pid->setTurnPid(true);
		pid->setTarget(targetAngle.Convert(radian));
	}
	
	OmniMotionProfiling::~OmniMotionProfiling()
	{
	}
	
} // namespace Pronounce
