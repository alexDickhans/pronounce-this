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
#include "motionProfiles/simpleSplineProfile.hpp"
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
		Pronounce::AbstractTankDrivetrain& drivetrain;
		Pronounce::PID turnPID, distancePID;
		std::function<double(QVelocity, QAcceleration)> feedforward;
		QLength startDistance;
		QTime startTime;
		std::function<Angle()> angleFunction;

		std::vector<std::pair<double, std::function<void()>>> commands;
		std::unordered_map<std::string, std::function<void()>> commandMap;
		int commandsIndex = 0;

		pros::Mutex movingMutex;
		std::pair<QVelocity, QVelocity> lastDriveVelocity;
	public:
		PathFollower(const std::shared_ptr<AbstractMotionProfile> &motionProfile,
		             Pronounce::AbstractTankDrivetrain &drivetrain, const Pronounce::PID &turnPid,
		             const Pronounce::PID &distancePid, std::function<double(QVelocity, QAcceleration)> feedforward,
		             const std::function<Angle()> &angleFunction) : motionProfile(motionProfile),
		                                                            drivetrain(drivetrain), turnPID(turnPid),
		                                                            distancePID(distancePid),
		                                                            feedforward(std::move(feedforward)),
		                                                            angleFunction(angleFunction) {}


		void setMotionProfile(const std::shared_ptr<AbstractMotionProfile> &motionProfile) {
			movingMutex.lock();
			PathFollower::motionProfile = motionProfile;
			movingMutex.give();
		}

		void addCommandMapping(const std::string& name, std::function<void()> function) {
			commandMap[name] = std::move(function);
		}

		void initialize() override {
			movingMutex.lock();
			startTime = pros::millis() * 1_ms;
			startDistance = drivetrain.getDistanceSinceReset();
			lastDriveVelocity = {0.0, 0.0};
			distancePID.reset();
			turnPID.reset();
			commandsIndex = 0;
		}

		void update() override {
			QTime time = pros::millis() * 1_ms - startTime;

			auto target = motionProfile->update(time - startTime);
			auto futureTarget = motionProfile->update(time - startTime + 10_ms);

			double index = target.targetT;

			if (commands.size() > commandsIndex) {
				if (commands.at(commandsIndex).first < index) {
					commands.at(commandsIndex).second();
					commandsIndex ++;
				}
			}

			std::pair<QVelocity, QVelocity> driveSpeeds = this->getChassisSpeeds(target.targetSpeed, target.targetCurvature);
			std::pair<QVelocity, QVelocity> futureDriveSpeeds = this->getChassisSpeeds(futureTarget.targetSpeed, futureTarget.targetCurvature);
			std::pair<QAcceleration, QAcceleration> driveAccel = {(futureDriveSpeeds.first - driveSpeeds.first)/10_ms, (futureDriveSpeeds.second - driveSpeeds.second)/10_ms};

			std::pair<double, double> driveVoltages = {feedforward(driveSpeeds.first, driveAccel.first), feedforward(driveSpeeds.second, driveAccel.second)};
			distancePID.setTarget(target.targetDistance.getValue());

			double distanceOutput = distancePID.update((drivetrain.getDistanceSinceReset() - startDistance).Convert(metre));
			driveVoltages = {driveVoltages.first + distanceOutput, driveVoltages.second + distanceOutput};

			turnPID.setTarget((target.targetPose.getAngle() + ((driveSpeeds.first + driveSpeeds.second).getValue() > 0.0 ? 0.0 : 180_deg)).Convert(radian));
			double turnOutput = turnPID.update(this->angleFunction().Convert(radian));
			driveVoltages = {driveVoltages.first + turnOutput, driveVoltages.second - turnOutput};

			drivetrain.tankSteerVoltage(driveVoltages.first, driveVoltages.second);
		}

		void exit() override {
			drivetrain.tankSteerVoltage(0.0, 0.0);
			movingMutex.unlock();
		}

		bool isDone() override {
			return pros::millis() * 1_ms - startTime > this->motionProfile->getDuration();
		}

		std::pair<QVelocity, QVelocity> getChassisSpeeds(QVelocity speed, QCurvature curvature) {
			return {speed.getValue() * (2.0 + curvature.getValue() * drivetrain.getTrackWidth().getValue()) / 2.0, speed.getValue() * (2.0 - curvature.getValue() * drivetrain.getTrackWidth().getValue()) / 2.0};
		}
	};
}
