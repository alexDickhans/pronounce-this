#pragma once

#include <vector>
#include "pid/pid.hpp"
#include "utils/path.hpp"
#include "utils/position.hpp"
#include "utils/vector.hpp"
#include "utils/purePursuitProfileManager.hpp"
#include "utils/purePursuitProfile.hpp"
#include "odometry/odometry.hpp"
#include "chassis/omniDrivetrain.hpp"

namespace Pronounce {
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

		OmniDrivetrain* drivetrain;
		Odometry* odometry;

		bool enabled = false;
		bool following = false;
	public:
		PurePursuit();
		PurePursuit(double lookahead);
		PurePursuit(OmniDrivetrain* drivetrain);
		PurePursuit(OmniDrivetrain* drivetrain, double lookahead);

		void addPath(Path path) {
			paths.emplace_back(path);
		}

		double getLookahead() {
			return lookahead;
		}

		void setLookahead(double lookahead) {
			this->lookahead = lookahead;
		}

		std::vector<Path> getPaths() {
			return this->paths;
		}

		Path getPath(int index) {
			return paths.at(index);
		}

		int getCurrentPathIndex() {
			return currentPath;
		}

		void setCurrentPathIndex(int index) {
			currentPath = index;
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

		OmniDrivetrain* getDrivetrain() {
			return drivetrain;
		}

		Odometry* getOdometry() {
			return odometry;
		}

		void setOdometry(Odometry* odometry) {
			this->odometry = odometry;
		}

		void setDrivetrain(OmniDrivetrain* drivetrain) {
			this->drivetrain = drivetrain;
		}

		bool isEnabled() {
			return enabled;
		}

		void setEnabled(bool enabled) {
			this->enabled = enabled;
			if (!enabled) {
				drivetrain->setDriveVectorVelocity(Vector(0.0, 0.0));
			}
		}

		bool isFollowing() {
			return following;
		}

		void setFollowing(bool following) {
			this->following = following;
		}

		void update();

		~PurePursuit();
	};
} // Pronounce