#pragma once

#include "utils/vector.hpp"
#include "utils/pose2d.hpp"
#include <exception>

namespace Pronounce {

	class PositionNotReady : public std::exception {
		const char* what() const throw () {
			return "Position not ready";
		}
	};

	/**
	 * @brief A class for odometry where the outputs are discontinuous or only gets a reliable position every once in a while
	 *
	 * @authors Alex Dickhans(alexDickhans)
	 */
	class InterruptOdometry {
	private:
	public:
		/**
		 * @brief Construct a new Interrupt Odometry object, run this in all your child objects
		 *
		 */
		InterruptOdometry() {}

		/**
		 * @brief Find out of the object is ready to call get position
		 *
		 * @param currentPose The current pose of the robot
		 * @param velocity The current velocity of the robot
		 * @return true The position is ready so call the get position function
		 * @return false The position is not ready so do not call the get position function
		 */
		virtual bool positionReady(Pose2D currentPose, Vector velocity) { return false; }

		/**
		 * @brief Get the refined position
		 *
		 * @param currentPose The current pose of the robot
		 * @param velocity The current velocity of the robot
		 * @return Pose2D The refined position of the robot
		 */
		virtual Pose2D getPosition(Pose2D currentPose, Vector velocity) {
			return currentPose;
		}

		~InterruptOdometry() {}
	};
} // namespace Pronounce
