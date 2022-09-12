#pragma once

#include "api.h"
#include "odometry/interruptOdometry/interruptOdometry.hpp"
#include <vector>
#include <exception>

namespace Pronounce
{
	class VisionOdometry : public InterruptOdometry {
	private:
		pros::Vision& visionSensor;
		std::pair<uint16_t, Point> target1;
		std::pair<uint16_t, Point> target2;

		QSpeed maxSpeed;

		Angle visionSensorOffset;

		int32_t lastXPixel;

	public:
		VisionOdometry(pros::Vision& visionSensor, QSpeed maxSpeed) : visionSensor(visionSensor) {

		}

		virtual bool positionReady(Pose2D currentPose, Vector velocity) {
			pros::vision_object_s_t signature;

			if (visionSensor.get_by_sig(0, target1.first).x_middle_coord != 0.0) {
				signature = visionSensor.get_by_sig(0, target1.first);
			}
			else {
				signature = visionSensor.get_by_sig(0, target2.first);
			}

			bool positionReady = visionSensor.get_object_count() > 0 && velocity.getMagnitude() < maxSpeed * 1_s && signature.x_middle_coord != lastXPixel;

			lastXPixel = signature.x_middle_coord;

			return positionReady;
		}

		virtual Pose2D getPosition(Pose2D currentPose, Vector velocity) {
			pros::vision_object_s_t signature;
			std::pair<uint16_t, Point> target;

			if (visionSensor.get_object_count() > 0) {
				if (visionSensor.get_by_sig(0, target1.first).x_middle_coord != 0.0) {
					target = target1;
					signature = visionSensor.get_by_sig(0, target1.first);
				}
				else {
					target = target2;
					signature = visionSensor.get_by_sig(0, target2.first);
				}
			} else {
				throw PositionNotReady();
			}

			// Map the pixel values to the angles on the vision sensor
			Angle angle = map(signature.x_middle_coord, -340.0, 340.0, -(30_deg).Convert(radian), (30_deg).Convert(radian));

			// Offset by the robot rotation and the vision sensor offset on the robot
			angle = (visionSensorOffset + currentPose.getAngle()) - angle;

			QLength distanceFromGoal = currentPose.distance(target.second);

			// Create a vector with the correct angle and distance
			Vector targetVector = Vector(distanceFromGoal, -angle);

			// Set the current pose to the new values from the vector
			currentPose = targetVector.getCartesian() + target.second;

			return currentPose;
		}

		~VisionOdometry();
	};
} // namespace Pronounce
