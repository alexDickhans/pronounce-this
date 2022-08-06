#pragma once

#include "interruptOdometry.hpp"
#include "utils/path.hpp"
#include "utils/pose2d.hpp"
#include "utils/pointUtil.hpp"
#include "sensors/lineSensorArray.hpp"

// TODO: Add docstrings
// TODO: Test code
// TODO: Add comments
// TODO: Clean up code
// TODO: Finish code

namespace Pronounce {
	class TapeOdometry {
	private:
		Point offset;

		LineSensorArray& lineSensors;

		// Configurations
		int32_t minLightDifference = 1;
		double maxDistance = 24;

		Path getClosestLine() {
			return Path();
		}

		double getClosestLineDistance(Point currentPosition) {
			// Have to find a cheap way to do this, we will be running this every frame
			return currentPosition.distance(this->getClosestLine().getClosestPoint(currentPosition));
		}

	public:
		TapeOdometry(Point offset, LineSensorArray& lineSensors) : lineSensors(lineSensors) {
			this->offset = offset;
		}

		virtual bool positionReady(Pose2D currentPose, Vector velocity) { 
			// Determines if the tapes have detected anything by checking if the difference between the 
			// max line sensor reading and the minimum line sensor reading is greater than the minimum.
			return lineSensors.getMin() + minLightDifference > lineSensors.getMax() && this->getClosestLineDistance(currentPose) < maxDistance;
		}

		virtual Pose2D getPosition(Pose2D currentPose, Vector velocity) {
			// Find which line it is, and find the point on this line that it is closest to
			Path closestLine = getClosestLine();

			// Reset odometry based off of that line
			Point closestPoint = this->getClosestLine().getClosestPoint(currentPose);

			// Add orientation to the closestPoint
			Pose2D newPose = Pose2D(closestPoint.getX(), closestPoint.getY(), currentPose.getAngle());

			// Return the result
			return newPose;
		}

		~TapeOdometry();
	};	
} // namespace Pronounce
