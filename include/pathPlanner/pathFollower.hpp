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
#include "json/asset.hpp"
#include "velocityProfile/trapezoidalVelocityProfile.hpp"

namespace PathPlanner {

	class PathFollower : public Pronounce::Behavior {
	private:
		std::vector<std::pair<BezierSegment, Pronounce::VelocityProfile*>> pathSegments;
		Pronounce::AbstractTankDrivetrain& drivetrain;
		Pronounce::ProfileConstraints defaultProfileConstraints;
		Pronounce::PID turnPID, distancePID;
		double feedforwardMultiplier;
		QLength startDistance;
		QTime startTime;
		std::function<Angle()> angleFunction;

		std::vector<std::pair<double, std::function<void()>>> commands;
		std::unordered_map<std::string, std::function<void()>> commandMap;
		int commandsIndex = 0;

		pros::Mutex movingMutex;
	public:
		PathFollower(std::string name, Pronounce::ProfileConstraints defaultProfileConstraints, Pronounce::AbstractTankDrivetrain& drivetrain, std::function<Angle()> angleFunction, const Pronounce::PID& turnPID, const Pronounce::PID& distancePID, double feedforwardMultiplier, QSpeed maxSpeed, std::initializer_list<std::pair<BezierSegment, Pronounce::VelocityProfile*>> pathSegments, std::initializer_list<std::pair<double, std::function<void()>>> functions = {}) : Pronounce::Behavior(std::move(name)), drivetrain(drivetrain) {
			this->pathSegments = pathSegments;
			this->commands = functions;
			this->turnPID = turnPID;
			this->turnPID.setTurnPid(true);
			this->distancePID = distancePID;
			this->feedforwardMultiplier = feedforwardMultiplier;
			this->angleFunction = std::move(angleFunction);
			this->defaultProfileConstraints = defaultProfileConstraints;

			calculate();
		}

		PathFollower(std::string name, Pronounce::ProfileConstraints defaultProfileConstraints, Pronounce::AbstractTankDrivetrain& drivetrain, std::function<Angle()> angleFunction, const Pronounce::PID& turnPID, const Pronounce::PID& distancePID, double feedforwardMultiplier, QSpeed maxSpeed, std::vector<std::pair<BezierSegment, Pronounce::VelocityProfile*>> pathSegments, std::initializer_list<std::pair<double, std::function<void()>>> functions = {}) : Pronounce::Behavior(std::move(name)), drivetrain(drivetrain) {
			this->pathSegments = std::move(pathSegments);
			this->commands = functions;
			this->turnPID = turnPID;
			this->turnPID.setTurnPid(true);
			this->distancePID = distancePID;
			this->feedforwardMultiplier = feedforwardMultiplier;
			this->angleFunction = std::move(angleFunction);
			this->defaultProfileConstraints = defaultProfileConstraints;

			calculate();
		}

		void addCommandMapping(const std::string& name, std::function<void()> function) {
			commandMap[name] = std::move(function);
		}

		PathFollower* changePath(Pronounce::ProfileConstraints defaultProfileConstraints, std::vector<std::pair<BezierSegment, Pronounce::VelocityProfile*>> path, std::initializer_list<std::pair<double, std::function<void()>>> functions = {}) {
			movingMutex.take();
			this->defaultProfileConstraints = defaultProfileConstraints;
			pathSegments = std::move(path);
			this->commands = functions;
			movingMutex.give();

			return calculate();
		}

		PathFollower* changePath(Pronounce::ProfileConstraints defaultProfileConstraints, std::vector<std::pair<BezierSegment, Pronounce::VelocityProfile*>> path, std::vector<std::pair<double, std::function<void()>>> functions) {
			movingMutex.take();
			this->defaultProfileConstraints = defaultProfileConstraints;
			pathSegments = std::move(path);
			this->commands = std::move(functions);
			movingMutex.give();

			return calculate();
		}

		PathFollower* changePath(asset path) {
			Json parsed_path = open_asset_as_json(path);

			std::vector<std::pair<BezierSegment, Pronounce::VelocityProfile*>> parsedPath;

			auto segments = parsed_path["segments"];

			for (auto segment : segments.array_items()) {
				auto constraints = segment["constraints"];
				auto paths = segment["paths"];

				parsedPath.emplace_back(
						BezierSegment(
								Point(paths[0]),
								Point(paths[1]),
								Point(paths[2]),
						        Point(paths[3]),
				                segment["inverted"].bool_value()),
							new Pronounce::SinusoidalVelocityProfile(0.0,
																	 Pronounce::ProfileConstraints{
							constraints["velocity"].number_value() * (inch/second).Convert(metre/second),
							constraints["accel"].number_value() * (inch/second/second).Convert(metre/second/second),
							0.0
							        }));
			}

			std::vector<std::pair<double, std::function<void()>>> functions;

			auto commands = parsed_path["commands"];

			for (auto command : commands.array_items()) {
					functions.emplace_back(
						command["t"].number_value(),
						commandMap.count(command["name"].string_value()) == 1 ? commandMap[command["name"].string_value()] : [&]() -> void {});
			}

			return changePath(defaultProfileConstraints, parsedPath, functions);
		}

		PathFollower* calculate() {
			movingMutex.take();

			for (int i = 0; i < this->pathSegments.size(); ++i) {

				QSpeed adjustedSpeed = this->pathSegments.at(i).first.getMaxSpeedMultiplier(drivetrain.getTrackWidth()) * drivetrain.getMaxSpeed();

				Pronounce::VelocityProfile* profile = this->pathSegments.at(i).second;

				if (profile == nullptr) {
					profile = new Pronounce::SinusoidalVelocityProfile(this->pathSegments.at(i).first.getDistance(), defaultProfileConstraints);
				}

				profile->setDistance(this->pathSegments.at(i).first.getDistance());

				profile->setProfileConstraints({std::min(profile->getProfileConstraints().maxVelocity.getValue(), adjustedSpeed.getValue()), profile->getProfileConstraints().maxAcceleration, 0.0});

				profile->setInitialSpeed(0.0);
				profile->setEndSpeed(0.0);

				if (i < this->pathSegments.size()-1) {
					if (this->pathSegments.at(i+1).first.getReversed() == this->pathSegments.at(i).first.getReversed())
						profile->setEndSpeed(profile->getProfileConstraints().maxVelocity.getValue());// * (this->pathSegments.at(i).first.getReversed() ? -1.0 : 1.0));
				}

				if (i > 0) {
					if (this->pathSegments.at(i-1).first.getReversed() == this->pathSegments.at(i).first.getReversed())
						profile->setInitialSpeed(this->pathSegments.at(i-1).second->getProfileConstraints().maxVelocity.getValue());// * (this->pathSegments.at(i).first.getReversed() ? -1.0 : 1.0));
				}

				profile->calculate();

				this->pathSegments.at(i).second = profile;
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
			double index = getIndex(time);

			if (commands.size() > commandsIndex) {
				if (commands.at(commandsIndex).first < index) {
					commands.at(commandsIndex).second();
					commandsIndex ++;
				}
			}

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

		void exit() override {
			drivetrain.tankSteerVoltage(0.0, 0.0);
		}

		bool isDone() override {
			return pros::millis() * 1_ms - startTime > totalTime();
		}

		double getIndex(QTime time) {
			int index = 0;

			while (time >= pathSegments.at(index).second->getDuration()) {
				time -= pathSegments.at(index).second->getDuration();
				index ++;
			}

			return index + pathSegments.at(index).first.getTByLength(abs(pathSegments.at(index).second->getDistanceByTime(time).getValue()));
		}

		QTime getTimeRemainder(QTime time) {
			int index = 0;

			while (time >= pathSegments.at(index).second->getDuration()) {
				time -= pathSegments.at(index).second->getDuration();
				index ++;
			}

			return time;
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
				index ++;
			}

			QCurvature curvature = pathSegments.at(index).first.getCurvature(pathSegments.at(index).first.getTByLength(abs(pathSegments.at(index).second->getDistanceByTime(time).getValue())));
			QSpeed speed = pathSegments.at(index).second->getVelocityByTime(time);

			return {speed.getValue() * (2.0 + curvature.getValue() * trackWidth.getValue()) / 2.0, speed.getValue() * (2.0 - curvature.getValue() * trackWidth.getValue()) / 2.0};
		}

		QLength getDistance(QTime time) {
			int index = 0;
			QLength totalDistance = 0.0;

			while (time >= pathSegments.at(index).second->getDuration()) {
				time -= pathSegments.at(index).second->getDuration();
				totalDistance += pathSegments.at(index).first.getDistance();
				index ++;
			}

			QLength distance = pathSegments.at(index).second->getDistanceByTime(time) + totalDistance;

			return distance;
		}

		Angle getAngle(QTime time) {
			int index = 0;

			while (time > pathSegments.at(index).second->getDuration()) {
				time -= pathSegments.at(index).second->getDuration();
				index ++;
			}

			return pathSegments.at(index).first.getAngle(abs(pathSegments.at(index).first.getTByLength(abs(pathSegments.at(index).second->getDistanceByTime(time).getValue()))));
		}
	};
}
