#pragma once

#include "continuousOdometry/continuousOdometry.hpp"
#include "units/units.hpp"
#include <vector>
#include <exception>

namespace Pronounce {
	class OdomFuser : public ContinuousOdometry {
	private:
		ContinuousOdometry& continuousOdometry;
	public:
		explicit OdomFuser(ContinuousOdometry& continuousOdometry) : continuousOdometry(continuousOdometry) { }

		void update() override {

			continuousOdometry.update();

			// Set the velocity to the continuous odometry because we don't want to have large jumps in the speed when it is reset
			this->setCurrentVelocity(continuousOdometry.getCurrentVelocity());

			// get the pose from the continuous odometry and use that as the baseline
			Pose2D currentPose = continuousOdometry.getPose();

			currentPose.log("CurrentPose");

			// Set the pose to the end result
			this->setPose(currentPose);
		}

		void reset(Pose2D pose) override {
			this->setPose(pose);
			this->setResetPose(pose);
			this->setCurrentVelocity(Vector());
		}
		
		~OdomFuser() {}
	};
} // namespace Pronounce
