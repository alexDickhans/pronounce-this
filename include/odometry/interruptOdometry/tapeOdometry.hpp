#pragma once

#include "interruptOdometry.hpp"
#include "utils/path.hpp"
#include "utils/pose2d.hpp"
#include "utils/pointUtil.hpp"
#include "sensors/lineSensorArray.hpp"

// TODO: Test code
// TODO: Add comments

namespace Pronounce {
	/**
	 * @brief Track the tapes on the field with an array of line sensors
	 * 
	 * @authors Alex Dickhans(alexDickhans)
	 */
	class TapeOdometry {
	private:
		/**
		 * @brief Reference to the line sensors
		 * 
		 */
		LineSensorArray& lineSensors;

		/**
		 * @brief Minimum light difference for a detection to occur
		 * 
		 */
		int32_t minLightDifference = 1;

		/**
		 * @brief If we are too far from the line don't use the reading
		 * 
		 */
		QLength maxDistance = 24.0;

		/**
		 * @brief All the lines on the field that we want to track
		 * 
		 */
		std::vector<Path> lines;

		/**
		 * @brief Calculate the current path object
		 * 
		 * @param currentPose The current pose of a robot
		 * @return Path The closest path to the robot
		 */
		Path getCurrentLine(Point currentPose) {
			// Add default values
			Path closestLine = this->lines.at(0);
			QLength closestDistance = this->lines.at(0).getClosestPoint(currentPose).distance(currentPose);

			// Loop through to find the closest line
			for (int i = 1; i < lines.size(); i++) {
				Path currentLine = this->lines.at(i);
				QLength distance = currentLine.getClosestPoint(currentPose).distance(currentPose);

				// Replace the values
				if (distance < closestDistance) {
					closestLine = currentLine;
					closestDistance = distance;
				}
			}
			
			// Return the results
			return closestLine;
		}

		QLength getCurrentDistance(Point currentPosition) {
			// Have to find a cheap way to do this, we will be running this every frame
			return currentPosition.distance(this->getCurrentLine(currentPosition).getClosestPoint(currentPosition));
		}

	public:
		/**
		 * @brief Construct a new Tape Odometry with lineSensors set
		 * 
		 * @param lineSensors Reference to the line sensor
		 */
		TapeOdometry(LineSensorArray& lineSensors) : lineSensors(lineSensors) {
		}

		/**
		 * @brief Get if a pose is ready
		 * 
		 * @param currentPose The current pose of the robot
		 * @param velocity The current velocity of the robot
		 * @return true The pose is ready
		 * @return false The pose isn't ready
		 */
		bool positionReady(Pose2D currentPose, Vector velocity) { 
			// Determines if the tapes have detected anything by checking if the difference between the 
			// max line sensor reading and the minimum line sensor reading is greater than the minimum.
			return lineSensors.getMin().second + minLightDifference > lineSensors.getMax().second && this->getCurrentDistance(currentPose) < maxDistance;
		}

		/**
		 * @brief Refine the current pose
		 * 
		 * @param currentPose The current pose of the robot
		 * @param velocity The current velocity of the robot
		 * @return Pose2D Refined pose of the robot
		 */
		Pose2D getPosition(Pose2D currentPose, Vector velocity) {

			std::pair<Point, int32_t> detection = this->lineSensors.getMax();

			// Find which line it is, and find the point on this line that it is closest to
			Vector sensorPosition = Vector(detection.first);
			sensorPosition.rotate(currentPose.getAngle());

			Point sensorPoint = currentPose + sensorPosition.getCartesian();

			Path closestLine = this->getCurrentLine(sensorPoint);

			// Reset odometry based off of that line
			Point closestPoint = closestLine.getClosestPoint(sensorPoint);

			// Add orientation to the closestPoint
			Pose2D newPose = Pose2D(closestPoint.getX(), closestPoint.getY(), currentPose.getAngle());

			// Return the result
			return newPose;
		}

		~TapeOdometry();
	};	
} // namespace Pronounce
