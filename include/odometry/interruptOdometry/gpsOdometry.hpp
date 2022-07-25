#pragma once

#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "api.h"

namespace Pronounce {
	class GpsOdometry : public ContinuousOdometry {
	private:
		pros::Gps* gps;

		pros::c::gps_status_s_t lastPos;
		bool goodFixBool = false;
		int32_t lastGoodFix = -10000;
		int32_t lastUpdate = 0;

		double gpsX = 0;
		double gpsY = 0;
		double gpsOrientation = 0;

		double convertToLocal(double x) {
			return (x - 1.8) * 39.37;
		}

		double convertToGlobal(double x) {
			return (x / 39.37) + 1.8;
		}
	public:
		GpsOdometry();
		GpsOdometry(pros::Gps* gps);

		void update();
		void update(Pose2D* pose);

		void reset(Pose2D* pose) {
			this->setPose(pose);
			this->setResetPose(pose);
			gps->initialize_full(convertToGlobal(pose->getX()), convertToGlobal(pose->getY()), toDegrees(pose->getAngle()) + gpsOrientation, gpsX, gpsY);
		}

		pros::Gps* getGps() {
			return gps;
		}

		void setGps(pros::Gps* gps) {
			this->gps = gps;
		}

		bool goodFix() {

			pros::c::gps_status_s_t currentPos = gps->get_status();

			if (pros::millis() - lastUpdate < 50) {
				if (goodFixBool && pros::millis() - lastGoodFix > 2000) {
					return true;
				}
				return false;
			}

			// If our position is 0 we invalidate it
			if (sqrt(pow(currentPos.x, 2) + pow(currentPos.y, 2)) == 0.0) {
				goodFixBool = false;
				return false;
			}

			// If the robot isn't moving too far start the timer
			if (sqrt(pow(currentPos.x - lastPos.x, 2) + pow(currentPos.y - lastPos.y, 2)) < (2.0 / (1000.0 / (double)(pros::millis() - lastUpdate))) && !goodFixBool) {
				if (!goodFixBool) {
					lastGoodFix = pros::millis();
				}
				goodFixBool = true;
			}
			else {
				goodFixBool = false;
				return false;
			}

			// Wait to return a good fix until we know that it is good for a while
			if (goodFixBool && pros::millis() - lastGoodFix > 2000) {
				return true;
			}

			return false;
		}

		~GpsOdometry();
	};
} // namespace Pronounce
