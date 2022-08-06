#pragma once

#include "odometry/interruptOdometry/interruptOdometry.hpp"
#include "api.h"
#include "units/units.hpp"

namespace Pronounce {
	class GpsOdometry : public InterruptOdometry {
	private:
		pros::Gps& gps;

		Pose2D lastPos;

		QLength gpsX = 0_m;
		QLength gpsY = 0_m;
		Angle gpsOrientation = 0_rad;

		QLength outsideBuffer = 24_in;

		QLength convertFromGps(QLength x) {
			return x - 1.8_m;
		}

		QLength convertToGps(QLength x) {
			return x + 1.8_m;
		}
	public:
		GpsOdometry(pros::Gps& gps, QLength gpsX, QLength gpsY, Angle gpsOrientation) : gps(gps), lastPos(0, 0, 0) {
			this->gpsX = gpsX;
			this->gpsY = gpsY;
			this->gpsOrientation = gpsOrientation;
		}

		bool positionReady(Pose2D currentPose, Vector velocity) {
			Pose2D pose = this->getPosition(currentPose, velocity);

			if (!(pose.getX() > outsideBuffer && pose.getX() < 144_in - outsideBuffer && pose.getY() > outsideBuffer && pose.getY() < 144_in - outsideBuffer)) {
				return false;
			}

			// TODO: Add more conditionals

			return true;

			lastPos = pose;
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
