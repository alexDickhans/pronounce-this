#pragma once

#include "stateMachine/behavior.hpp"
#include "utils/bezierSegment.hpp"
#include <utility>
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "feedbackControllers/pid.hpp"
#include "feedbackControllers/feedbackController.hpp"
#include "api.h"
#include <math.h>
#include "utils/utils.hpp"

namespace PathPlanner {

	class PathFollower : public Pronounce::Behavior {
	private:
		std::vector<std::pair<BezierSegment, Pronounce::SinusoidalVelocityProfile*>> pathSegments;
		Pronounce::AbstractTankDrivetrain& drivetrain;
		Pronounce::PID turnPID, distancePID;
		double feedforwardMultiplier;
		QLength startDistance;
		QTime startTime;
		std::function<Angle()> angleFunction;
	public:
		PathFollower(Pronounce::ProfileConstraints defaultProfileConstraints, Pronounce::AbstractTankDrivetrain& drivetrain, std::function<Angle()> angleFunction, const Pronounce::PID& turnPID, const Pronounce::PID& distancePID, double feedforwardMultiplier, QSpeed maxSpeed, std::initializer_list<std::pair<BezierSegment, Pronounce::SinusoidalVelocityProfile*>> pathSegments) : drivetrain(drivetrain) {
			this->pathSegments = pathSegments;
			this->turnPID = turnPID;
			this->turnPID.setTurnPid(true);
			this->distancePID = distancePID;
			this->feedforwardMultiplier = feedforwardMultiplier;
			this->angleFunction = std::move(angleFunction);

			for (int i = 0; i < this->pathSegments.size(); ++i) {

				QSpeed adjustedSpeed = this->pathSegments.at(i).first.getMaxSpeedMultiplier(drivetrain.getTrackWidth()) * maxSpeed;

				Pronounce::SinusoidalVelocityProfile* profile = this->pathSegments.at(i).second;

				if (profile == nullptr) {
					profile = new Pronounce::SinusoidalVelocityProfile(this->pathSegments.at(i).first.getDistance(), defaultProfileConstraints);
				}

				profile->setDistance(this->pathSegments.at(i).first.getDistance());

				if (abs(profile->getProfileConstraints().maxVelocity.getValue()) > adjustedSpeed.getValue()) {
					profile->setProfileConstraints({adjustedSpeed, profile->getProfileConstraints().maxAcceleration, profile->getProfileConstraints().maxJerk});
				}

				profile->setInitialSpeed(0.0);
				profile->setEndSpeed(0.0);

				if (i < this->pathSegments.size()-1 && this->pathSegments.at(i+1).second->getDistance().getValue() == this->pathSegments.at(i).second->getDistance().getValue()) {
					profile->setEndSpeed(profile->getProfileConstraints().maxVelocity);
				}

				if (i > 0 && this->pathSegments.at(i-1).first.getReversed() == this->pathSegments.at(i).first.getReversed()) {
					profile->setInitialSpeed(this->pathSegments.at(i-1).second->getProfileConstraints().maxVelocity);
				}

				profile->calculate(20);

				this->pathSegments.at(i).second = profile;
			}
		}

		void initialize() override {
			startTime = pros::millis() * 1_ms;
			startDistance = drivetrain.getDistanceSinceReset();
			distancePID.reset();
			turnPID.reset();
		}

		void update() override {
			QTime time = pros::millis() * 1_ms - startTime;

			std::pair<QSpeed, QSpeed> driveSpeeds = this->getChassisSpeeds(time, drivetrain.getTrackWidth());

			std::pair<double, double> driveVoltages = {driveSpeeds.first.Convert(inch/second) * feedforwardMultiplier, driveSpeeds.second.Convert(inch/second) * feedforwardMultiplier};
			if ((driveSpeeds.first + driveSpeeds.second).getValue() < 0.0) {
				driveVoltages = {driveVoltages.second, driveVoltages.first};
			}
			distancePID.setTarget(this->getDistance(time).getValue());

			double distanceOutput = distancePID.update((drivetrain.getDistanceSinceReset() - startDistance).Convert(metre));
			driveVoltages = {driveVoltages.first + distanceOutput, driveVoltages.second + distanceOutput};

			turnPID.setTarget((this->getAngle(time) + ((driveSpeeds.first + driveSpeeds.second).getValue() > 0.0 ? 0.0 : 180_deg)).Convert(radian));
			double turnOutput = turnPID.update(this->angleFunction().Convert(radian));
			driveVoltages = {driveVoltages.first + turnOutput, driveVoltages.second - turnOutput};

			drivetrain.tankSteerVoltage(driveVoltages.first, driveVoltages.second);
		}

		void exit() {
			drivetrain.tankSteerVoltage(0.0, 0.0);
		}

		bool isDone() {
			return pros::millis() * 1_ms - startTime > totalTime();
		}

		QTime totalTime() {
			QTime totalTime = 0.0;

			std::for_each(pathSegments.begin(), pathSegments.end(), [&](const auto &item) {
				totalTime += item.second->getDuration();
			});

			return totalTime;
		}

		std::pair<QSpeed, QSpeed> getChassisSpeeds(QTime time, QLength trackWidth) {
			int index = 0;
			QLength totalDistance = 0.0;

			while (time >= pathSegments.at(index).second->getDuration()) {
				time -= pathSegments.at(index).second->getDuration();
				totalDistance += pathSegments.at(index).first.getDistance();
			}

			QCurvature curvature = pathSegments.at(index).first.getCurvature(pathSegments.at(index).first.getTByLength(pathSegments.at(index).second->getDistanceByTime(time)));
			QLength distance = pathSegments.at(index).second->getDistanceByTime(time) + totalDistance;
			QSpeed speed = pathSegments.at(index).second->getVelocityByTime(time);

			return {speed.getValue() * (2.0 + curvature.getValue() * trackWidth.getValue()) / 2.0, speed.getValue() * (2.0 - curvature.getValue() * trackWidth.getValue()) / 2.0};
		}

		QLength getDistance(QTime time) {
			int index = 0;
			QLength totalDistance = 0.0;

			while (time >= pathSegments.at(index).second->getDuration()) {
				time -= pathSegments.at(index).second->getDuration();
				totalDistance += pathSegments.at(index).first.getDistance();
			}

			QCurvature curvature = pathSegments.at(index).first.getCurvature(pathSegments.at(index).first.getTByLength(pathSegments.at(index).second->getDistanceByTime(time)));
			QLength distance = pathSegments.at(index).second->getDistanceByTime(time) + totalDistance;
			QSpeed speed = pathSegments.at(index).second->getVelocityByTime(time);

			return distance;
		}

		Angle getAngle(QTime time) {
			int index = 0;

			while (time > pathSegments.at(index).second->getDuration()) {
				time -= pathSegments.at(index).second->getDuration();
			}

			return pathSegments.at(index).first.getAngle(pathSegments.at(index).first.getTByLength(pathSegments.at(index).second->getDistanceByTime(time)));
		}
	};
}
