#pragma once

#include <vector>
#include <math.h>
#include "pid/pid.hpp"
#include "utils/path.hpp"
#include "utils/position.hpp"
#include "utils/utils.hpp"
#include "utils/vector.hpp"
#include "utils/purePursuitProfileManager.hpp"
#include "utils/purePursuitProfile.hpp"
#include "odometry/odometry.hpp"
#include "chassis/omniDrivetrain.hpp"

namespace Pronounce {
	struct PurePursuitPointData {
		Point lookaheadPoint;
		Vector lookaheadVector;
		Vector normalizedLookaheadVector;
		double curvature;
	};

	/**
	 * @brief Abstract class for tracking paths, read full docstring for impelmentation details
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
		double stopDistance;
		double normalizeDistance = 1;

		PurePursuitProfile currentProfile;

		PurePursuitProfileManager purePursuitProfileManager;

		double turnTarget;

		Odometry* odometry;

		PurePursuitPointData pointData;

		bool enabled = false;
		bool following = false;
	public:
		PurePursuit();
		PurePursuit(double lookahead);
		PurePursuit(OmniDrivetrain* drivetrain);
		PurePursuit(OmniDrivetrain* drivetrain, double lookahead);

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
			this->currentProfile = purePursuitProfileManager.getProfile(currentPath);
		}

		double getNormalizeDistance() {
			return normalizeDistance;
		}

		void setNormalizeDistance(double normalizeDistance) {
			this->normalizeDistance = normalizeDistance;
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

		~PurePursuit();
	};
} // Pronounce