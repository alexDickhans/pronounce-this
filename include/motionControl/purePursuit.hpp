#pragma once

#include <vector>
#include <math.h>
#include <algorithm>
#include <iostream>
#include "feedbackControllers/pid.hpp"
#include "utils/path.hpp"
#include "utils/position.hpp"
#include "utils/utils.hpp"
#include "utils/vector.hpp"
#include "utils/purePursuitProfileManager.hpp"
#include "utils/purePursuitProfile.hpp"
#include "odometry/odometry.hpp"

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
	class PurePursuit {
	private:
		double lookahead;
		std::vector<Path> paths;
		int currentPath = -1;
		double stopDistance = 0;
		double normalizeDistance = 1;

		double maxAcceleration = 0;
		double maxDecelleration = 0;

		PurePursuitProfile currentProfile;

		PurePursuitProfileManager purePursuitProfileManager;

		double turnTarget;

		Odometry* odometry;

		PurePursuitPointData pointData;

		bool enabled = false;
		bool following = false;

		bool atPoint = false;
	public:
		PurePursuit();
		PurePursuit(double lookahead);
		PurePursuit(Odometry* odometry, double lookahead);

		virtual void updateDrivetrain() {}

		void updatePointData();

		void update() {
			if (!enabled)
				return;

			if (paths.size() == 0 || currentPath == -1 || (paths.size() <= currentPath) || !following) {
				this->stop();
				return;
			}

			updatePointData();
			updateDrivetrain();
		}

		virtual void stop() {}

		int addPath(Path path) {
			paths.emplace_back(path);
			printf("%d\n", paths.size());
			std::cout << "Name: " << path.getName() << std::endl;
			// printf(path.to_string().c_str());
			return paths.size() - 1;
		}

		std::vector<Path> getPaths() {
			return this->paths;
		}

		PurePursuitPointData getPointData() {
			return this->pointData;
		}

		Path getPath(int index) {
			return paths.at(index);
		}

		int getCurrentPathIndex() {
			return currentPath;
		}

		double getTurnTarget() {
			return turnTarget;
		}

		void setTurnTarget(double turnTarget) {
			this->turnTarget = turnTarget;
		}

		void setCurrentPathIndex(int index) {
			currentPath = index;
			printf("Current Path: %d\n", currentPath);
			printf("Current path tostring: %s\n", paths.at(currentPath).to_string().c_str());
			this->currentProfile = purePursuitProfileManager.getProfile(currentPath);
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

		PurePursuitProfileManager getPurePursuitProfileManager() {
			return purePursuitProfileManager;
		}

		void setPurePursuitProfileManager(PurePursuitProfileManager purePursuitProfileManager) {
			this->purePursuitProfileManager = purePursuitProfileManager;
		}

		Odometry* getOdometry() {
			return odometry;
		}

		void setOdometry(Odometry* odometry) {
			this->odometry = odometry;
		}

		bool isEnabled() {
			return enabled;
		}

		void setEnabled(bool enabled) {
			this->enabled = enabled;
			if (!enabled) {
				this->stop();
			}
		}

		bool isFollowing() {
			return following;
		}

		void setFollowing(bool following) {
			this->following = following;
		}

		bool isDone(double maxDistance) {
			return maxDistance > odometry->getPosition()->distance(paths.at(currentPath).getPoint(paths.at(currentPath).getPath().size() - 1));
		}

		double getStopDistance() {
			return stopDistance;
		}

		void setStopDistance(double stopDistance) {
			this->stopDistance = stopDistance;
		}

		double getLookahead() {
			return lookahead;
		}

		void setLookahead(double lookahead) {
			this->lookahead = lookahead;
		}

		double getMaxAcceleration() {
			return maxAcceleration;
		}

		void setMaxAcceleration(double maxAcceleration) {
			this->maxAcceleration = maxAcceleration;
		}

		~PurePursuit();
	};
} // Pronounce
