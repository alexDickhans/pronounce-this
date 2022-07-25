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
		double distanceFromEnd;
	};

	struct PurePursuitProfile {
		PID* lateralPid;
		PID* orientationPid;
		double lookaheadDistance;
		double maxAcceleration;
		double speed;
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
		Path path;
		double stopDistance = 0;
		double normalizeDistance = 1;

		double outputMultiplier = 1.0;

		PurePursuitProfile currentProfile;

		double turnTarget;

		ContinuousOdometry* odometry;

		double doneDistance = 0.5;

		PurePursuitPointData pointData;

		double updateTime = 20;
	public:
		PurePursuit();
		PurePursuit(double lookahead);
		PurePursuit(ContinuousOdometry* odometry, double lookahead);

		void initialize() {
			currentProfile.lateralPid->reset();
			currentProfile.orientationPid->reset();
		}

		virtual void updateDrivetrain() {}

		void updatePointData();

		void update() {
			updatePointData();
			updateDrivetrain();
		}

		virtual void exit() {}

		bool isDone(double maxDistance) {
			return maxDistance > odometry->getPosition()->distance(path.getPoint(path.getPath().size() - 1));
		}

		bool isDone() {
			return isDone(doneDistance);
		}

		double getOutputMultiplier() {
			return outputMultiplier;
		}

		void setOutputMultiplier(double outputMultiplier) {
			this->outputMultiplier = outputMultiplier;
		}

		Path getPath() {
			return path;
		}

		PurePursuitPointData getPointData() {
			return this->pointData;
		}

		double getTurnTarget() {
			return turnTarget;
		}

		void setTurnTarget(double turnTarget) {
			this->turnTarget = turnTarget;
		}

		double getNormalizeDistance() {
			return normalizeDistance;
		}

		void setNormalizeDistance(double normalizeDistance) {
			this->normalizeDistance = normalizeDistance;
		}

		PurePursuitProfile getCurrentProfile() {
			return this->currentProfile;
		}

		void setCurrentProfile(PurePursuitProfile profile) {
			this->currentProfile = profile;
		}

		ContinuousOdometry* getOdometry() {
			return odometry;
		}

		void setOdometry(ContinuousOdometry* odometry) {
			this->odometry = odometry;
		}

		double getStopDistance() {
			return stopDistance;
		}

		void setStopDistance(double stopDistance) {
			this->stopDistance = stopDistance;
		}

		double getDoneDistance() {
			return doneDistance;
		}

		void setDoneDistance(double doneDistance) {
			this->doneDistance = doneDistance;
		}

		double getUpdateTime() {
			return updateTime;
		}

		void setUpdateTime(double updateTime) {
			this->updateTime = updateTime;
		} 

		~PurePursuit();
	};
} // Pronounce
