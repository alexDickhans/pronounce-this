#pragma once

#include "velocityProfile/velocityProfile.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "stateMachine/behavior.hpp"
#include "time/robotTime.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "utils/path/combinedPath.hpp"

double velocityFeedforward = 180.0;
double accelerationFeedforward = 20;

namespace Pronounce {
	class TankMotionProfiling : public Behavior {
	private:
		pros::Mutex& drivetrainMutex;

		TankDrivetrain* drivetrain;
		VelocityProfile* velocityProfile;

		QTime startTime = 0.0;

		ContinuousOdometry* odometry;

		Angle targetAngle = 0.0;

		FeedbackController* distancePid;
		PID* turnPid = nullptr;

		CombinedPath path;

		pros::motor_brake_mode_e_t beforeBrakeMode;

		QLength startDistance;

	public:
		TankMotionProfiling(
				std::string name,
				TankDrivetrain* drivetrain,
				VelocityProfile* velocityProfile,
				ContinuousOdometry* odometry,
				FeedbackController* distancePid,
				pros::Mutex& drivetrainMutex) :
					Behavior(name),
					drivetrain(drivetrain),
					velocityProfile(velocityProfile),
					odometry(odometry),
					drivetrainMutex(drivetrainMutex),
					distancePid(distancePid) {

		}

		TankMotionProfiling(
				std::string name,
				TankDrivetrain* drivetrain,
				ProfileConstraints profileConstraints,
				QLength distance,
				ContinuousOdometry* odometry,
				FeedbackController* distancePid,
				pros::Mutex& drivetrainMutex,
				QCurvature curvature,
				QSpeed initialSpeed = 0.0,
				QSpeed endSpeed = 0.0) :
					Behavior(std::move(name)),
					drivetrain(drivetrain),
					odometry(odometry),
					drivetrainMutex(drivetrainMutex),
					distancePid(distancePid) {
			velocityProfile = new SinusoidalVelocityProfile(fabs(distance.getValue()), profileConstraints, initialSpeed, endSpeed);
			velocityProfile->calculate(100);
			this->path = CombinedPath();
			path.addPathSegment({distance, curvature});
		}

		TankMotionProfiling(
				std::string name,
				TankDrivetrain* drivetrain,
				ProfileConstraints profileConstraints,
				QLength distance,
				ContinuousOdometry* odometry,
				PID* distancePid,
				pros::Mutex& drivetrainMutex,
				QCurvature curvature,
				Angle targetAngle,
				PID* turnPid,
				QSpeed initialSpeed = 0.0,
				QSpeed endSpeed = 0.0) :
					Behavior(std::move(name)),
					drivetrain(drivetrain),
					odometry(odometry),
					drivetrainMutex(drivetrainMutex),
					targetAngle(targetAngle),
					turnPid(turnPid),
					distancePid(distancePid) {
			velocityProfile = new SinusoidalVelocityProfile(distance, profileConstraints, initialSpeed, endSpeed);
			velocityProfile->calculate(100);
			this->path = CombinedPath();
			path.addPathSegment({distance, curvature});
		}

		TankMotionProfiling(
				std::string name,
				TankDrivetrain* drivetrain,
				ProfileConstraints profileConstraints,
				CombinedPath path,
				ContinuousOdometry* odometry,
				PID* distancePid,
				pros::Mutex& drivetrainMutex,
				Angle targetAngle,
				PID* turnPid,
				QSpeed initialSpeed = 0.0,
				QSpeed endSpeed = 0.0) :
					Behavior(std::move(name)),
					drivetrain(drivetrain),
					odometry(odometry),
					drivetrainMutex(drivetrainMutex),
					targetAngle(targetAngle),
					turnPid(turnPid),
					distancePid(distancePid),
					path(std::move(path)) {
			velocityProfile = new SinusoidalVelocityProfile(this->path.getDistance(), profileConstraints, initialSpeed, endSpeed);
		}

		void initialize() {
			startTime = currentTime();

			startDistance = drivetrain->getDistanceSinceReset();

			QSpeed adjustedSpeed = path.getSpeedMultiplier(drivetrain->getTrackWidth()) * this->velocityProfile->getProfileConstraints().maxVelocity.getValue();

			this->velocityProfile->setDistance(path.getDistance());
			this->velocityProfile->setProfileConstraints({adjustedSpeed, velocityProfile->getProfileConstraints().maxAcceleration, velocityProfile->getProfileConstraints().maxJerk});

			this->velocityProfile->calculate(100);

			beforeBrakeMode = drivetrain->getBrakeMode();
			drivetrain->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
		}

		void update() override {
			QTime duration = currentTime() - startTime;

			drivetrainMutex.take();

			QAcceleration acceleration = velocityProfile->getAccelerationByTime(duration).getValue() * (path.getInverted() ? -1 : 1);
			QSpeed speed = velocityProfile->getVelocityByTime(duration).getValue() * (path.getInverted() ? -1 : 1);
			QLength distance = velocityProfile->getDistanceByTime(duration).getValue() * (path.getInverted() ? -1 : 1);

			drivetrain->targetSpeed = speed;
			drivetrain->targetDistance = distance + startDistance;

			double turnPower = 0;

			QLength currentDistance = (drivetrain->getDistanceSinceReset()-startDistance);
			QCurvature curvature = path.getSegmentAtDistance(fabs(distance.getValue())).curvature;

			distancePid->setTarget(distance.getValue());

			double wheelVoltage = distancePid->update(currentDistance.getValue()) + velocityFeedforward * speed.Convert(inch/second) + accelerationFeedforward * acceleration.Convert(inch/second/second);

			// add curvature
			double leftCurvatureAdjustment = (2.0 + curvature.getValue() * drivetrain->getTrackWidth().getValue()) / 2.0;
			double rightCurvatureAdjustment = (2.0 - curvature.getValue() * drivetrain->getTrackWidth().getValue()) / 2.0;

			double leftVoltage = leftCurvatureAdjustment * wheelVoltage;
			double rightVoltage = rightCurvatureAdjustment * wheelVoltage;

			// integrate turnPid
			if (turnPid != nullptr) {
				Angle offset = path.getAngleAtDistance(distance).getValue() * (path.getInverted() ? -1 : 1);

				Angle targetAngleWithOffset = targetAngle + offset;

				turnPid->setTarget(targetAngleWithOffset.getValue());
				
				turnPower = turnPid->update(odometry->getAngle().getValue());

				leftVoltage += turnPower;
				rightVoltage -= turnPower;
			}

			// send to motors
			drivetrain->tankSteerVoltage(leftVoltage, rightVoltage);

			drivetrainMutex.give();
		}

		void exit() override {
			drivetrainMutex.take();
			
			drivetrain->setBrakeMode(beforeBrakeMode);

			drivetrain->tankSteerVoltage(0.0, 0.0);

			drivetrainMutex.give();
		}

		bool isDone() {
			return currentTime() - startTime > velocityProfile->getDuration();
		}

		~TankMotionProfiling() {}
	};
} // namespace Pronounce
