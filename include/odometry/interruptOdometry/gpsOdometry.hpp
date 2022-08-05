#pragma once

#include "odometry/interruptOdometry/interruptOdometry.hpp"
#include "api.h"
#include "units/units.hpp"

namespace Pronounce {
	class GpsOdometry : public InterruptOdometry {
	private:
		pros::Gps& gps;

		Pose2D lastPos;
		bool goodFixBool = false;
		QTime lastGoodFix = -10000.0;
		QTime lastUpdate = 0.0;

		QLength gpsX = 0_m;
		QLength gpsY = 0_m;
		Angle gpsOrientation = 0_rad;

		QLength convertFromGps(QLength x) {
			return x - 1.8_m;
		}

		QLength convertToGps(QLength x) {
			return x + 1.8_m;
		}
	public:
		GpsOdometry(pros::Gps& gps) : gps(gps), lastPos(0, 0, 0) {
			
		}

		bool positionReady(Pose2D currentPose, Vector velocity) { 
			pros::c::gps_status_s_t currentPos = gps.get_status();

			// Convert to our coordinate space
			Pose2D currentPose = Pose2D(currentPos.x * 1_m, currentPos.y * 1_m);

			if ((pros::millis() * 1_ms) - lastUpdate < 50_ms) {
				if (goodFixBool && (pros::millis() * 1_ms) - lastGoodFix > 2000_ms) {
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
			if (sqrt(pow((currentPose.getX() - lastPos.getX()).getValue(), 2) + pow((currentPose.getY() - lastPos.getY()).getValue(), 2)) < (2.0 / (1000.0 / (double)(pros::millis() * 1_ms - lastUpdate))) && !goodFixBool) {
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

		virtual Pose2D getPosition(Pose2D currentPose, Vector velocity) {
			pros::c::gps_status_s_t currentReading = gps.get_status();
			Pose2D readingPose = Pose2D(convertFromGps(currentReading.x), convertFromGps(currentReading.y));

			return readingPose;
		}

		void reset(Pose2D pose) {
			gps.initialize_full(convertToGps(pose.getX()).getValue(), convertToGps(pose.getY()).getValue(), (pose.getAngle()+ gpsOrientation).Convert(degree), gpsX.getValue(), gpsY.getValue());
		}

		pros::Gps& getGps() {
			return gps;
		}

		~GpsOdometry() {}
	};
} // namespace Pronounce
