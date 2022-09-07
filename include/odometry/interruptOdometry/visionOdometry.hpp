#pragma once

#include "api.h"
#include "odometry/interruptOdometry/interruptOdometry.hpp"
#include <vector>

namespace Pronounce
{
	class VisionOdometry : public InterruptOdometry {
	private:
		pros::Vision& visionSensor;
		std::pair<uint16_t, Point> target1;
		std::pair<uint16_t, Point> target2;
	public:
		VisionOdometry(pros::Vision& visionSensor) : visionSensor(visionSensor) {

		}

		virtual bool positionReady(Pose2D currentPose, Vector velocity) {
			pros::vision_object_s_t signature;
			if (visionSensor.get_object_count() > 0) {
				signature = visionSensor.get_by_sig(0, target1.first).x_middle_coord != 0.0 ? visionSensor.get_by_sig(0, target1.first) : visionSensor.get_by_sig(0, target2.first);
			}

			Angle angle = (30_deg).getValue() * (signature.x_middle_coord * 320.0) ;


		}

		virtual Pose2D getPosition(Pose2D currentPose, Vector velocity) {
			return currentPose;
		}

		~VisionOdometry();
	};	
} // namespace Pronounce
