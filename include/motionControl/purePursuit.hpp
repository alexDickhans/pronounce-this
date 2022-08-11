#pragma once

#include <vector>
#include <math.h>
#include <algorithm>
#include <iostream>
#include "feedbackControllers/pid.hpp"
#include "utils/path.hpp"
#include "utils/pose2d.hpp"
#include "utils/utils.hpp"
#include "utils/vector.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
	/**
	 * @brief Struct to store the information that is used to control the robot.
	 * 
	 */
	struct PurePursuitPointData {
		Point lookaheadPoint;
		Vector lookaheadVector;
		Vector localLookaheadVector;
		Vector normalizedLookaheadVector;
		double curvature;
		QLength distanceFromEnd;
	};

	/**
	 * @brief Struct to hold all the values for a pure pursuit value
	 * 
	 */
	struct PurePursuitProfile {
		QLength lookaheadDistance;
		QAcceleration maxAcceleration;
		QSpeed speed;
	};

	/**
	 * @brief Abstract class for tracking paths. Read full docstring for impelmentation details
	 * 
	 * @attention PurePursuit an abstract class for tracking paths. 
	 * It is meant to be inherited by a class that implements the 
	 * PurePursuit::updateDrivetrain(), and PurePursuit::stop() methods.
	 * 
	 * @details PurePursuit::UpdateDrivetrain() is called every loop
	 * iteration after setting PointData, and is responsible 
	 * for sending inputs to the drivetrain, given a PurePursuitPointData
	 * struct.
	 * 
	 * @details PurePursuit::Stop() is called when the drivetrain motors 
	 * should be stopped.
	 * 
	 * @authors @ad101-lab
	 */
	class PurePursuit : public Behavior {
	private:
		/**
		 * @brief The current path that the robot is following
		 * 
		 */
		Path path;

		/**
		 * @brief The distance from the target to stop the robot at
		 * 
		 */
		QLength stopDistance = 1_in;

		/**
		 * @brief The current pure pursuit profile
		 * 
		 */
		PurePursuitProfile currentProfile;

		/**
		 * @brief The pointer to the odometry variable
		 * 
		 */
		ContinuousOdometry* odometry;

		/**
		 * @brief The update time, used for velocity calculation
		 * 
		 */
		QTime updateTime = 10_ms;
	public:
		/**
		 * @brief Default constructor
		 * 
		 */
		PurePursuit();

		/**
		 * @brief Construct a new Pure Pursuit class with the lookahead
		 * 
		 * @param lookahead The lookahead distances
		 */
		PurePursuit(QLength lookahead);

		/**
		 * @brief Construct a new Pure Pursuit class with the lookahead
		 * 
		 * @param odometry A pointer to the odometry object
		 * @param lookahead The lookahead distance
		 */
		PurePursuit(ContinuousOdometry* odometry, QLength lookahead);

		/**
		 * @brief Start all the values
		 * 
		 */
		void initialize() {
		}

		/**
		 * @brief update the drivetrain, implemented in the child classes
		 * 
		 */
		virtual void updateDrivetrain(PurePursuitPointData pointData) {}

		/**
		 * @brief Update the point data sent to the drivetrain
		 * 
		 */
		PurePursuitPointData updatePointData();

		/**
		 * @brief Call every frame at frametime intervals
		 * 
		 */
		void update() {
			updateDrivetrain(updatePointData());
		}

		/**
		 * @brief Leave the state
		 * 
		 */
		virtual void exit() {}

		/**
		 * @brief Return a boolean if the path is done
		 * 
		 * @param maxDistance The max distance from the target
		 * @return true it is within maxDistance of the end
		 * @return false It isn't within maxDistance of the end
		 */
		bool isDone(QLength maxDistance) {
			return maxDistance > odometry->getPosition()->distance(path.getPoint(path.getPath().size() - 1));
		}

		/**
		 * @brief Get the current Path
		 * 
		 * @return Path The current path
		 */
		Path getPath() {
			return path;
		}

		/**
		 * @brief Get the Current Profile object
		 * 
		 * @return PurePursuitProfile The current profile object
		 */
		PurePursuitProfile getCurrentProfile() {
			return this->currentProfile;
		}

		/**
		 * @brief Set the Current Profile object
		 * 
		 * @param profile The desired profile
		 */
		void setCurrentProfile(PurePursuitProfile profile) {
			this->currentProfile = profile;
		}

		/**
		 * @brief Get the Odometry pointer
		 * 
		 * @return ContinuousOdometry* Pointer to the odometry object
		 */
		ContinuousOdometry* getOdometry() {
			return odometry;
		}

		/**
		 * @brief Set the Odometry Pointer
		 * 
		 * @param odometry new odometry pointer
		 */
		void setOdometry(ContinuousOdometry* odometry) {
			this->odometry = odometry;
		}

		/**
		 * @brief Get the Stop Distance 
		 * 
		 * @return QLength The distance to stop at
		 */
		QLength getStopDistance() {
			return stopDistance;
		}

		/**
		 * @brief Set the Stop Distance object
		 * 
		 * @param stopDistance The desired distance for the robot to stop at
		 */
		void setStopDistance(double stopDistance) {
			this->stopDistance = stopDistance;
		}

		/**
		 * @brief Get the Update Time variable
		 * 
		 * @return QTime The current update time
		 */
		QTime getUpdateTime() {
			return updateTime;
		}

		/**
		 * @brief Set the Update Time
		 * 
		 * @param updateTime The current update time
		 */
		void setUpdateTime(QTime updateTime) {
			this->updateTime = updateTime;
		} 

		~PurePursuit();
	};
} // Pronounce
