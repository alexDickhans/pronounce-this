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
//			movingMutex.take();

			if (pathSegments.empty())
				return this;

			std::vector<Eigen::Vector2d> limitedVelocities;

			QLength distance = 0.0;
			QLength totalDistance = 0.0;
			QTime lastTime = 0.0;
			bool lastInverted = pathSegments[0].first.getReversed();
			profiles.clear();
			distanceToT.add(0, 0);

			for (int i = 0; i < this->pathSegments.size(); i++) {
				int granularity = std::floor(std::max(5.0, abs(pathSegments.at(i).first.getDistance().Convert(inch))));
				QLength distanceInterval = abs(pathSegments.at(i).first.getDistance().getValue()) / static_cast<double>(granularity);

				if (lastInverted != pathSegments[i].first.getReversed()) {
					profiles.emplace_back(limitedVelocities, defaultProfileConstraints, lastInverted, lastTime, totalDistance-distance);
					lastInverted = pathSegments[i].first.getReversed();
					limitedVelocities.clear();
					lastTime = profiles[profiles.size() - 1].getEndTime();
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

			profiles.emplace_back(limitedVelocities, defaultProfileConstraints, lastInverted, lastTime);

//			movingMutex.give();

			return this;
		}

	public:
		PathFollower(std::string name, Eigen::Vector3d defaultProfileConstraints, Pronounce::AbstractTankDrivetrain& drivetrain, std::function<Angle()> angleFunction, const Pronounce::PID& turnPID, const Pronounce::PID& distancePID, std::function<double(QSpeed, QAcceleration)> feedforwardFunction, std::initializer_list<std::pair<BezierSegment, QSpeed>> pathSegments, std::initializer_list<std::pair<double, std::function<void()>>> functions = {}) : Pronounce::Behavior(std::move(name)), drivetrain(drivetrain) {
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
			this->defaultProfileConstraints = std::move(defaultProfileConstraints);
			pathSegments = std::move(path);
			this->commands = functions;

			return calculate();
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
				while (commands.at(commandsIndex).first < index) {
					commands.at(commandsIndex).second();
					commandsIndex ++;
				}
			}

			Eigen::Vector<QSpeed, 2> driveSpeeds = this->getChassisSpeeds(time, drivetrain.getTrackWidth());
			Eigen::Vector<QSpeed, 2> nextDriveSpeeds = this->getChassisSpeeds(time + 10_ms, drivetrain.getTrackWidth());

			Eigen::Vector<QSpeed, 2> difference = nextDriveSpeeds - driveSpeeds;
			Eigen::Vector<QAcceleration, 2> acceleration = {difference(0) / 10_ms, difference(1) / 10_ms};
			Eigen::Vector2d driveVoltages = {feedforwardFunction(driveSpeeds(0), acceleration(0)), feedforwardFunction(driveSpeeds(1), acceleration(1))};

			if (pathSegments.at(index).first.getReversed()) {
				Eigen::Matrix2d conversionMatrix {
					{0, 1},
					{1, 0}
				};
				driveVoltages = conversionMatrix * driveVoltages;
			}

			distancePID.setTarget(this->getDistance(time).getValue());

			std::cout << "CURRENT-DISTANCE: " << this->getDistance(time).Convert(inch) << std::endl;

			double distanceOutput = distancePID.update((drivetrain.getDistanceSinceReset() - startDistance).Convert(metre));
			driveVoltages = driveVoltages + Eigen::Vector2d(distanceOutput, distanceOutput);

			turnPID.setTarget((this->getAngle(time) + (pathSegments.at(index).first.getReversed() ? 180.0_deg : 0_deg)).Convert(radian));
			double turnOutput = turnPID.update(this->angleFunction().Convert(radian));
			driveVoltages = driveVoltages + Eigen::Vector2d(turnOutput, -turnOutput);

			drivetrain.tankSteerVoltage(driveVoltages(0), driveVoltages(1));
		}

		void exit() override {
			drivetrain.tankSteerVoltage(0.0, 0.0);
		}

		bool isDone() override {
			return pros::millis() * 1_ms - startTime > totalTime();
		}

		double getPathIndex(QTime time) {
			int index;

			for (index = 0; time > profiles[index].getEndTime() && index < profiles.size(); index ++);

			QLength distance = 0.0;

			for (int i = 0; i < index; i ++) {
				distance += profiles[i].getLength();
			}

			distance += profiles[index].getRawDistance(time);

			std::cout << "HIIdistance: " << distance.Convert(inch) << " Time: " << time.getValue() << std::endl;

			return std::clamp(distanceToT.get(distance.getValue()), 0.0, static_cast<double>((pathSegments.size())) - 0.0001);
		}

		QTime totalTime() {
			return profiles[profiles.size()-1].getEndTime();
		}

		Eigen::Vector<QSpeed, 2> getChassisSpeeds(QTime time, QLength trackWidth) {
			int profileIndex;

			for (profileIndex = 0; time > profiles[profileIndex].getEndTime() && profileIndex < profiles.size(); profileIndex ++);

			double pathIndex = getPathIndex(time);

			QCurvature curvature = pathSegments.at((int) pathIndex).first.getCurvature(pathIndex - std::floor(pathIndex));
			Eigen::Vector2d speed = {profiles[profileIndex].getSpeed(time).getValue(), profiles[profileIndex].getSpeed(time).getValue()};
			std::cout << "CURRENT-SPEED: " << speed(0) * metre.Convert(inch) << std::endl;
			Eigen::Matrix2d curvatureAdjustment {{1.0 + (curvature.getValue() * trackWidth.getValue() * 0.5), 0}, {1.0 -(curvature.getValue() * trackWidth.getValue() * 0.5)}};
			speed = curvatureAdjustment * speed;

			return {speed(0), speed(1)};
		}

		QLength getDistance(QTime time) {
			int profileIndex;

			for (profileIndex = 0; time > profiles[profileIndex].getEndTime() && profileIndex < profiles.size(); profileIndex ++);

			QLength distance = profiles[profileIndex].getDistance(time);

			return distance;
		}

		Angle getAngle(QTime time) {
			double pathIndex = getPathIndex(time);

			std::cout << "HII: turntarget: " << (turnPID.getTarget() * 1_rad).Convert(degree) << " index: " << pathIndex << std::endl;

			return pathSegments.at((int) pathIndex).first.getAngle(pathIndex - std::floor(pathIndex));
		}
	};
}
