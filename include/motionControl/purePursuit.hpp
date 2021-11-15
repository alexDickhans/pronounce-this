#pragma once

#include <vector>
#include "pid/pid.hpp"
#include "utils/path.hpp"
#include "utils/position.hpp"
#include "odometry/odometry.hpp"
#include "chassis/omniDrivetrain.hpp"

namespace Pronounce {
	class PurePursuit {
	private:
		double lookahead;
		std::vector<Path> paths;
		int currentPath = -1;
		double stopDistance;

		PID* lateralPid;
		PID* anglePid;

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
		}

		PID* getLateralPid() {
			return lateralPid;
		}

		void setLateralPid(PID* lateralPid) {
			this->lateralPid = lateralPid;
		}

		PID* getAnglePid() {
			return anglePid;
		}

		void setAnglePid(PID* anglePid) {
			this->anglePid = anglePid;
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