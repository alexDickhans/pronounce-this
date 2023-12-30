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
		std::vector<std::pair<BezierSegment, Eigen::Vector3d>> pathSegments;
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
	public:
		PathFollower(std::string name, Eigen::Vector3d defaultProfileConstraints, Pronounce::AbstractTankDrivetrain& drivetrain, std::function<Angle()> angleFunction, const Pronounce::PID& turnPID, const Pronounce::PID& distancePID, std::function<double(QSpeed, QAcceleration)> feedforwardFunction, std::initializer_list<std::pair<BezierSegment, Eigen::Vector3d>> pathSegments, std::initializer_list<std::pair<double, std::function<void()>>> functions = {}) : Pronounce::Behavior(std::move(name)), drivetrain(drivetrain) {
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

		PathFollower* changePath(Eigen::Vector3d defaultProfileConstraints, std::vector<std::pair<BezierSegment, Eigen::Vector3d>> path, std::initializer_list<std::pair<double, std::function<void()>>> functions = {}) {
			movingMutex.take();
			this->defaultProfileConstraints = std::move(defaultProfileConstraints);
			pathSegments = std::move(path);
			this->commands = functions;
			movingMutex.give();

			return calculate();
		}

		PathFollower* calculate() {
			movingMutex.take();

			auto limitedVelocities = std::vector<Eigen::Vector2d>({{0.0, 0.0}});

			QLength distance = 0.0;
			QLength totalDistance = 0.0;
			QTime lastTime = 0.0;
			bool lastInverted = pathSegments.begin()->first.getReversed();
			profiles.clear();

			for (int i = 0; i < this->pathSegments.size(); ++i) {
				int granularity = std::floor(std::max(5.0, pathSegments.at(i).first.getDistance().Convert(inch)));

				distance = 0.0;

				if (lastInverted != pathSegments[i].first.getReversed()) {
					profiles.emplace_back(limitedVelocities, defaultProfileConstraints, lastInverted, lastTime);
					lastInverted = pathSegments[i].first.getReversed();
					limitedVelocities.clear();
					lastTime = profiles[profiles.size() - 1].getEndTime();
				}

				for (int j = 0; j <= granularity; j++) {
					distance += pathSegments.at(i).first.getDistance() / static_cast<double>(granularity);
					totalDistance += pathSegments.at(i).first.getDistance() / static_cast<double>(granularity);
					distanceToT.add(totalDistance.getValue(), (double) i + this->pathSegments[i].first.getTByLength(distance));

					if (pathSegments[i].second.isZero()) {
						pathSegments[i].second = defaultProfileConstraints;
					}

					limitedVelocities.emplace_back(distance.getValue(), std::min(drivetrain.getMaxSpeed().getValue() * this->pathSegments[i].first.getSpeedMultiplier(this->pathSegments[i].first.getTByLength(distance), drivetrain.getTrackWidth()), pathSegments[i].second(0, 0)));
				}
			}

			movingMutex.give();

			return this;
		}

		void initialize() override {
			startTime = pros::millis() * 1_ms;
			startDistance = drivetrain.getDistanceSinceReset();
			distancePID.reset();
			turnPID.reset();
			commandsIndex = 0;
		}

		void update() override {
			QTime time = pros::millis() * 1_ms - startTime;
			double index = getPathIndex(time);

			if (commands.size() > commandsIndex) {
				if (commands.at(commandsIndex).first < index) {
					commands.at(commandsIndex).second();
					commandsIndex ++;
				}
			}

			std::pair<QSpeed, QSpeed> driveSpeeds = this->getChassisSpeeds(time, drivetrain.getTrackWidth());
			std::pair<double, double> driveVoltages = {feedforwardFunction(driveSpeeds.first, 0.0), feedforwardFunction(driveSpeeds.second, 0.0)};
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

		void exit() override {
			drivetrain.tankSteerVoltage(0.0, 0.0);
		}

		bool isDone() override {
			return pros::millis() * 1_ms - startTime > totalTime();
		}

		double getPathIndex(QTime time) {
			int index = 0;

			for (index = 0; time > profiles[index].getEndTime() && index < profiles.size(); index ++);

			QLength distance = 0.0;

			for (int i = 0; i < index; i ++) {
				distance += profiles[i].getLength();
			}

			distance += profiles[index].getRawDistance(time);

			return distanceToT.get(distance.getValue());
		}

		QTime totalTime() {
			return profiles[profiles.size()-1].getEndTime();
		}

		std::pair<QSpeed, QSpeed> getChassisSpeeds(QTime time, QLength trackWidth) {
			int profileIndex;

			for (profileIndex = 0; time > profiles[profileIndex].getEndTime() && profileIndex < profiles.size(); profileIndex ++);

			double pathIndex = getPathIndex(time);

			QCurvature curvature = pathSegments.at((int) pathIndex).first.getCurvature(pathIndex - std::floor(pathIndex));
			QSpeed speed = profiles[profileIndex].getSpeed(time);

			return {speed.getValue() * (2.0 + curvature.getValue() * trackWidth.getValue()) / 2.0, speed.getValue() * (2.0 - curvature.getValue() * trackWidth.getValue()) / 2.0};
		}

		QLength getDistance(QTime time) {
			int profileIndex;

			for (profileIndex = 0; time > profiles[profileIndex].getEndTime() && profileIndex < profiles.size(); profileIndex ++);

			QLength distance = profiles[profileIndex].getDistance(time);

			return distance;
		}

		Angle getAngle(QTime time) {
			double pathIndex = getPathIndex(time);

			return pathSegments.at((int) pathIndex).first.getAngle(pathIndex - std::floor(pathIndex));
		}
	};
}
