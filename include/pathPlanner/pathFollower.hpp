#pragma once

#include "stateMachine/behavior.hpp"
#include "utils/bezierSegment.hpp"
#include <utility>
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "chassis/tankDrive.hpp"
#include "feedbackControllers/pid.hpp"
#include "feedbackControllers/feedbackController.hpp"
#include "api.h"
#include <cmath>
#include "motionProfiles/abstractMotionProfile.hpp"
#include "utils/utils.hpp"
#include "json/asset.hpp"
#include "velocityProfile/trapezoidalVelocityProfile.hpp"
#include "motionProfiles/smoothSplineProfile.hpp"
#include "motionProfiles/combinedMotionProfile.hpp"

namespace PathPlanner {

	class PathFollower : public Pronounce::Behavior {
	private:
		std::shared_ptr<AbstractMotionProfile> motionProfile;
		Pronounce::TankDrivetrain& drivetrain;
		Pronounce::PID turnPID, distancePID;
		std::function<double(QVelocity, QAcceleration)> feedforward;
		QLength startDistance;
		QTime startTime;
		std::function<Angle()> angleFunction;

		std::vector<std::pair<double, std::function<void()>>> commands;
		std::unordered_map<std::string, std::function<void()>> commandMap;
		int commandsIndex = 0;
	public:
		PathFollower(const std::shared_ptr<AbstractMotionProfile> &motionProfile,
		             Pronounce::TankDrivetrain &drivetrain, const Pronounce::PID &turnPid,
		             const Pronounce::PID &distancePid, std::function<double(QVelocity, QAcceleration)> feedforward,
		             const std::function<Angle()> &angleFunction) : Pronounce::Behavior("PathFollower"), motionProfile(motionProfile),
		                                                            drivetrain(drivetrain), turnPID(turnPid),
		                                                            distancePID(distancePid),
		                                                            feedforward(std::move(feedforward)),
		                                                            angleFunction(angleFunction) {}


		void setMotionProfile(const std::shared_ptr<AbstractMotionProfile> &motionProfile) {
			PathFollower::motionProfile = motionProfile;
			commands.clear();
			for (const auto &command: motionProfile->getCommands()) {
				if (commandMap.count(command.second) > 0) {
					Log(string_format("%f, %s", command.first, command.second.c_str()));
					commands.emplace_back(command.first, commandMap[command.second]);
				}
			}
		}

		void addCommandMapping(const std::string& name, std::function<void()> function) {
			commandMap[name] = std::move(function);
		}

		void initialize() override {
			startTime = pros::millis() * 1_ms;
			startDistance = drivetrain.getDistanceSinceReset();
			distancePID.reset();
			turnPID.reset();
			commandsIndex = 0;
			Log("Init");
		}

		void update() override {
			QTime time = (pros::millis() * 1_ms) - startTime;
			auto target = motionProfile->update(time);
			auto futureTarget = motionProfile->update(time + 10_ms);

			double index = target.targetT;
			Log(std::to_string(index));

			for(; commands.size() > commandsIndex && commands.at(commandsIndex).first <= index; commandsIndex++) {
				commands.at(commandsIndex).second();
			}

			std::pair<QVelocity, QVelocity> driveSpeeds = this->getChassisSpeeds(target.targetSpeed, target.targetCurvature);

			std::pair<QVelocity, QVelocity> futureDriveSpeeds = this->getChassisSpeeds(futureTarget.targetSpeed, futureTarget.targetCurvature);
			std::pair<QAcceleration, QAcceleration> driveAccel = {(futureDriveSpeeds.first - driveSpeeds.first)/10_ms, (futureDriveSpeeds.second - driveSpeeds.second)/10_ms};

			std::pair<double, double> driveVoltages = {feedforward(driveSpeeds.first, driveAccel.first), feedforward(driveSpeeds.second, driveAccel.second)};
			distancePID.setTarget(target.targetDistance.getValue());

			double distanceOutput = std::clamp(distancePID.update((drivetrain.getDistanceSinceReset() - startDistance).Convert(metre)), -12000.0, 12000.0);
			driveVoltages = {driveVoltages.first + distanceOutput, driveVoltages.second + distanceOutput};

			turnPID.setTarget((target.targetPose.getAngle() + ((driveSpeeds.first + driveSpeeds.second).getValue() > 0.0 ? 0.0 : 180_deg)).Convert(radian));
			double turnOutput = turnPID.update(this->angleFunction().Convert(radian));
			driveVoltages = {driveVoltages.first + turnOutput, driveVoltages.second - turnOutput};

			drivetrain.tankSteerVoltage(driveVoltages.first, driveVoltages.second);
		}

		void exit() override {
			Log("exit");
			drivetrain.tankSteerVoltage(0.0, 0.0);
		}

		bool isDone() override {
			return (pros::millis() * 1_ms - startTime) > this->motionProfile->getDuration();
		}

		std::pair<QVelocity, QVelocity> getChassisSpeeds(QVelocity speed, QCurvature curvature) {
			return {speed.getValue() * (2.0 + curvature.getValue() * drivetrain.getTrackWidth().getValue()) / 2.0, speed.getValue() * (2.0 - curvature.getValue() * drivetrain.getTrackWidth().getValue()) / 2.0};
		}
	};
}
