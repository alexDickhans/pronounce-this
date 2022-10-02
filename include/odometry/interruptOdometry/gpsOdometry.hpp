#pragma once

#include "odometry/interruptOdometry/interruptOdometry.hpp"
#include "api.h"
#include "units/units.hpp"

// TODO: Add more logic to determine if the pose that we get is good

namespace Pronounce {
	
	/**
	 * @brief Use the gps to rebase odometry measurements
	 * 
	 * @authors Alex Dickhans (alexDickhans)
	 */
	class GpsOdometry : public InterruptOdometry {
	private:
		/**
		 * @brief Refrence to the current gps object
		 * 
		 */
		pros::Gps& gps;

		/**
		 * @brief Last pose reading from the gps
		 * 
		 */
		Pose2D lastPos;

		/**
		 * @brief Position of the gps on the robot
		 * 
		 */
		QLength gpsX = 0_m, gpsY = 0_m;

		/**
		 * @brief Angle of the gps on the robot
		 * 
		 */
		Angle gpsOrientation = 0_rad;

		/**
		 * @brief How far from the wall the gps needs to reset odometry
		 * 
		 */
		QLength outsideBuffer = 24_in;

		/**
		 * @brief Convert value from gps coordinates to global coordinates 
		 * 
		 * @param x Gps coordinates
		 * @return QLength Global coordinates
		 */
		QLength convertFromGps(QLength x) {
			return x - 1.8_m;
		}

		/**
		 * @brief Convert value from global coordinates to gps coordinates
		 * 
		 * @param x Global coordinates
		 * @return QLength Gps coordinates
		 */
		QLength convertToGps(QLength x) {
			return x + 1.8_m;
		}
	public:
		/**
		 * @brief Construct a new Gps Odometry object
		 * 
		 * @param gps Refrence to gps object
		 * @param gpsX Gps x position on the robot
		 * @param gpsY Gps y position on the robot
		 * @param gpsOrientation Gps orientation on the robot
		 */
		GpsOdometry(pros::Gps& gps, QLength gpsX, QLength gpsY, Angle gpsOrientation) : gps(gps), lastPos(0.0, 0.0, 0.0) {
			this->gpsX = gpsX;
			this->gpsY = gpsY;
			this->gpsOrientation = gpsOrientation;
		}

		/**
		 * @brief Get if the gps has a valid pose ready
		 * 
		 * @param currentPose The current pose of the robot
		 * @param velocity The curent velocity of the robot
		 * @return true If the readings are valid 
		 * @return false If the readings are not valid
		 */
		bool positionReady(Pose2D currentPose, Vector velocity) {
			Pose2D pose = this->getPosition(currentPose, velocity);

			if (!(pose.getX() > outsideBuffer && pose.getX() < 144_in - outsideBuffer && pose.getY() > outsideBuffer && pose.getY() < 144_in - outsideBuffer)) {
				return false;
			}

			// TODO: Add more conditionals

			return true;

			lastPos = pose;
		}

		/**
		 * @brief Get the current position of the gps
		 * 
		 * @param currentPose The current pose of the robot, use in the calculations
		 * @param velocity The current velocity of the robot, used in the calculations
		 * @return Pose2D The refined pose of the robot
		 */
		Pose2D getPosition(Pose2D currentPose, Vector velocity) {
			pros::c::gps_status_s_t currentReading = gps.get_status();
			Pose2D readingPose = Pose2D(convertFromGps(currentReading.x), convertFromGps(currentReading.y));

			return readingPose;
		}

		/**
		 * @brief Get a reference to the gps object
		 * 
		 * @return pros::Gps& Reference to the gps object
		 */
		pros::Gps& getGps() {
			return gps;
		}

		~GpsOdometry() {}
	};
} // namespace Pronounce
