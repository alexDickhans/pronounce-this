#pragma once

#include "continuousOdometry/continuousOdometry.hpp"
#include "units/units.hpp"
#include "interruptOdometry/interruptOdometry.hpp"
#include <vector>
#include <exception>

// TODO: Add comments
// TODO: Add docstrings
// TODO: check if this works with velocity

namespace Pronounce {
	class OdomFuser : public ContinuousOdometry {
	private:
		ContinuousOdometry& continuousOdometry;
		std::vector<InterruptOdometry*> interruptOdometrys;
	public:
		OdomFuser(ContinuousOdometry& continuousOdometry) : continuousOdometry(continuousOdometry) { }

		void update() {

			continuousOdometry.update();

			// Set the velocity to the continuous odometry because we don't want to have large jumps in the speed when it is reset
			this->setCurrentVelocity(continuousOdometry.getCurrentVelocity());

			// get the pose from the continuous odometry and use that as the baseline
			Pose2D currentPose = continuousOdometry.getPose();

			// Go through each of the interrupt odoms in a list, the sequential order selected by the user will allow the more accurate odometry types to go last and result in the best positioning
			for (int i = 0; i < interruptOdometrys.size(); i++) {
				if (interruptOdometrys.at(i)->positionReady(currentPose, this->getCurrentVelocity())) {
					try {
						currentPose = Pose2D(interruptOdometrys.at(i)->getPosition(currentPose, this->getCurrentVelocity()));
					} catch (std::exception e) {
						std::cout << "Interrupt position not ready. Index: " << i << std::endl;
					}
				}
			}

			currentPose.log("CurrentPose");

			// Set the pose to the end result
			this->setPose(currentPose);
		}

		void reset(Pose2D pose) {
			this->setPose(pose);
			this->setResetPose(pose);
			this->setCurrentVelocity(Vector());
		}
		
		~OdomFuser() {}
	};
} // namespace Pronounce
