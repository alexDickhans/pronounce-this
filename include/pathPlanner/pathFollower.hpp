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
#include <cmath>
#include "utils/distanceLimitedVelocityProfile.hpp"
#include "utils/utils.hpp"

namespace PathPlanner {

	class PathFollower : public Pronounce::Behavior {
	private:
		std::vector<std::pair<BezierSegment, QSpeed>> pathSegments;
		std::vector<DistanceLimitedTrapezoidalProfile> profiles;
		LinearInterpolator distanceToT;
		Pronounce::AbstractTankDrivetrain& drivetrain;
		Eigen::Vector3d defaultProfileConstraints;
		Pronounce::PID turnPID, distancePID;
		std::function<double(QSpeed, QAcceleration)> feedforwardFunction;
		QLength startDistance;
		QTime startTime;
		std::function<Angle()> angleFunction;

		std::vector<std::pair<double, std::function<void()>>> commands;
		int commandsIndex = 0;

		pros::Mutex movingMutex;

		PathFollower* calculate() {

			if (pathSegments.empty()) {
				movingMutex.give();
				return this;
			}

			std::vector<Eigen::Vector2d> limitedVelocities;

			QLength distance = 0.0;
			QLength totalDistance = 0.0;
			QLength lastDistance = 0.0;
			QTime lastTime = 0.0;
			bool lastInverted = pathSegments[0].first.getReversed();
			profiles.clear();
			distanceToT.clear();
			distanceToT.add(0, 0);

			for (int i = 0; i < this->pathSegments.size(); i++) {
				int granularity = std::floor(std::max(5.0, abs(pathSegments.at(i).first.getDistance().Convert(1_in))));
				QLength distanceInterval = abs(pathSegments.at(i).first.getDistance().getValue()) / static_cast<double>(granularity);

				std::cout << "CALC-Length: " << pathSegments.at(i).first.getDistance().Convert(inch) << " Granularity: " << granularity << std::endl;

				if (lastInverted != pathSegments[i].first.getReversed()) {
					profiles.emplace_back(limitedVelocities, defaultProfileConstraints, lastInverted, lastTime, lastDistance);
					lastInverted = pathSegments[i].first.getReversed();
					limitedVelocities.clear();
					lastTime = profiles[profiles.size() - 1].getEndTime();
					lastDistance = profiles[profiles.size() - 1].getEndDistance();
					distance = 0.0;
				}

				if (pathSegments[i].second.getValue() == 0.0) {
					pathSegments[i].second = defaultProfileConstraints(0, 0);
				}

				QLength currentPathLength = 0.0;

				for (int j = 0; j < granularity; j++) {
					limitedVelocities.emplace_back(distance.getValue(), std::min(drivetrain.getMaxSpeed().getValue() * this->pathSegments[i].first.getSpeedMultiplier(this->pathSegments[i].first.getTByLength(currentPathLength), drivetrain.getTrackWidth()), pathSegments[i].second.getValue()));

					distance += distanceInterval;
					totalDistance += distanceInterval;
					currentPathLength += distanceInterval;

					distanceToT.add(totalDistance.getValue(), (double) i + this->pathSegments[i].first.getTByLength(distance));
				}
			}

			profiles.emplace_back(limitedVelocities, defaultProfileConstraints, lastInverted, lastTime, lastDistance);

			std::for_each(profiles.begin(), profiles.end(), [&](auto &item) {
				std::cout << "CALC-profile: " << item.getEndDistance().Convert(inch) << " time: " << item.getEndTime().Convert(second) << std::endl;
			});

			for (QTime i = 0.0; i < this->totalTime(); i += 0.1_s) {
				std::cout << "CALC-time: " << i.Convert(second) << " displacement: " << this->getDistance(i).Convert(inch) << std::endl;
			}

			movingMutex.give();

			return this;
		}

	public:
		PathFollower(std::string name, Eigen::Vector3d defaultProfileConstraints, Pronounce::AbstractTankDrivetrain& drivetrain, std::function<Angle()> angleFunction, const Pronounce::PID& turnPID, const Pronounce::PID& distancePID, std::function<double(QSpeed, QAcceleration)> feedforwardFunction, std::initializer_list<std::pair<BezierSegment, QSpeed>> pathSegments, std::initializer_list<std::pair<double, std::function<void()>>> functions = {}) : Pronounce::Behavior(std::move(name)), drivetrain(drivetrain) {
			movingMutex.take(TIMEOUT_MAX);
			this->pathSegments = pathSegments;
			this->commands = functions;
			this->turnPID = turnPID;
			this->turnPID.setTurnPid(true);
			this->distancePID = distancePID;
			this->feedforwardFunction = std::move(feedforwardFunction);
			this->angleFunction = std::move(angleFunction);
			this->defaultProfileConstraints = std::move(defaultProfileConstraints);

			calculate();
		}

		PathFollower* changePath(Eigen::Vector3d defaultProfileConstraints, std::vector<std::pair<BezierSegment, QSpeed>> path, std::initializer_list<std::pair<double, std::function<void()>>> functions = {}) {
			movingMutex.take(TIMEOUT_MAX);
			this->defaultProfileConstraints = std::move(defaultProfileConstraints);
			pathSegments = std::move(path);
			this->commands = functions;

			return calculate();
		}

		void initialize() override {
			movingMutex.take(TIMEOUT_MAX);
			startTime = pros::millis() * 1_ms;
			startDistance = drivetrain.getDistanceSinceReset();
			distancePID.reset();
			turnPID.reset();
			commandsIndex = 0;
			movingMutex.give();
		}

		void update() override {
			movingMutex.take(TIMEOUT_MAX);
			QTime time = pros::millis() * 1_ms - startTime;

			double pathIndex = getPathIndex(time);

			if (commands.size() > commandsIndex) {
				while (commands.size() > commandsIndex && commands.at(commandsIndex).first < pathIndex) {
					commands.at(commandsIndex).second();
					commandsIndex ++;
				}
			}

			std::cout << "ERROR: start update" << std::endl;
			Eigen::Vector<QSpeed, 2> driveSpeeds = this->getChassisSpeeds(time, drivetrain.getTrackWidth());
			Eigen::Vector<QSpeed, 2> nextDriveSpeeds = this->getChassisSpeeds(time + 10_ms, drivetrain.getTrackWidth());

			std::cout << "ERROR: start update" << std::endl;
			Eigen::Vector<QSpeed, 2> difference = nextDriveSpeeds - driveSpeeds;
			Eigen::Vector<QAcceleration, 2> acceleration = {difference(0) / 10_ms, difference(1) / 10_ms};
			Eigen::Vector2d driveVoltages = {feedforwardFunction(driveSpeeds(0), acceleration(0)), feedforwardFunction(driveSpeeds(1), acceleration(1))};

			distancePID.setTarget(this->getDistance(time).getValue());

			double distanceOutput = distancePID.update((drivetrain.getDistanceSinceReset() - startDistance).Convert(metre));
			driveVoltages = driveVoltages + Eigen::Vector2d(distanceOutput, distanceOutput);

			turnPID.setTarget((this->getAngle(pathIndex) + (profiles[getProfileIndex(time)].isReversed() ? 180.0_deg : 0_deg)).Convert(radian));
			double turnOutput = turnPID.update(this->angleFunction().Convert(radian));
			driveVoltages = driveVoltages + Eigen::Vector2d(turnOutput, -turnOutput);

			drivetrain.tankSteerVoltage(driveVoltages(0), driveVoltages(1));
			movingMutex.give();
		}

		void exit() override {
			movingMutex.take(TIMEOUT_MAX);
			drivetrain.tankSteerVoltage(0.0, 0.0);
			movingMutex.give();
		}

		bool isDone() override {
			return pros::millis() * 1_ms - startTime > totalTime();
		}

		QTime totalTime() {
			return profiles[profiles.size()-1].getEndTime();
		}

		int getProfileIndex(QTime time) {
			int profileIndex = 0;

			for (auto item : profiles) {
				if (time > item.getEndTime()) {
					profileIndex ++;
				}
			}

			return profileIndex;
		}

		double getPathIndex(QTime time) {
			QLength distanceTraveled = 0.0;
			int profileIndex = getProfileIndex(time);

			for (int i = 0; i < profileIndex; i++) {
				distanceTraveled += profiles[i].getLength();
			}

			distanceTraveled += profiles[profileIndex].getRawDistance(time);

			return std::min(static_cast<double>(pathSegments.size()) - 0.001, distanceToT.get(distanceTraveled.getValue()));
		}

		Eigen::Vector<QSpeed, 2> getChassisSpeeds(QTime time, QLength trackWidth) {

			std::cout << "ERROR: start getChassisSpeeds" << std::endl;

			int profileIndex = getProfileIndex(std::min(time, totalTime()));
			double pathIndex = getPathIndex(time);

			QCurvature curvature = pathSegments.at(std::floor(pathIndex)).first.getCurvature(pathIndex - std::floor(pathIndex));

			Eigen::Vector2d speed = {profiles[profileIndex].getSpeed(time).getValue(), profiles[profileIndex].getSpeed(time).getValue()};
			Eigen::Matrix2d curvatureAdjustment {{1.0 + (curvature.getValue() * trackWidth.getValue() * 0.5), 0}, {0, 1.0 -(curvature.getValue() * trackWidth.getValue() * 0.5)}};
			speed = curvatureAdjustment * speed;

			if (profiles[profileIndex].isReversed()) {
				speed = {speed(1), speed(0)};
			}

			std::cout << "ERROR: FINISH getChassisSpeeds" << std::endl;

			return {speed(0), speed(1)};
		}

		QLength getDistance(QTime time) {
			int profileIndex;

			for (profileIndex = 0; time > profiles[profileIndex].getEndTime() && profileIndex < profiles.size(); profileIndex ++);

			QLength distance = profiles[profileIndex].getDistance(time);

			return distance;
		}

		Angle getAngle(double pathIndex) {

			return pathSegments.at((int) pathIndex).first.getAngle(pathIndex - std::floor(pathIndex));
		}
	};
}
