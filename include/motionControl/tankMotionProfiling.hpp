#pragma once

#include "velocityProfile/velocityProfile.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "stateMachine/behavior.hpp"
#include "time/robotTime.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"

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

		QLength distance;
		QCurvature curvature;

		pros::MotorBrake beforeBrakeMode;

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
				QVelocity initialSpeed = 0.0,
				QVelocity endSpeed = 0.0) :
					Behavior(std::move(name)),
					drivetrain(drivetrain),
					odometry(odometry),
					drivetrainMutex(drivetrainMutex),
					distancePid(distancePid) {
			velocityProfile = new SinusoidalVelocityProfile(fabs(distance.getValue()), profileConstraints, initialSpeed, endSpeed);
			velocityProfile->calculate();
			this->distance = distance;
			this->curvature = curvature;
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
				QVelocity initialSpeed = 0.0,
				QVelocity endSpeed = 0.0) :
					Behavior(std::move(name)),
					drivetrain(drivetrain),
					odometry(odometry),
					drivetrainMutex(drivetrainMutex),
					targetAngle(targetAngle),
					turnPid(turnPid),
					distancePid(distancePid) {
			velocityProfile = new SinusoidalVelocityProfile(distance, profileConstraints, initialSpeed, endSpeed);
			velocityProfile->calculate();
			this->distance = distance;
			this->curvature = curvature;
		}

		double getSpeedMultiplier() {

			if (curvature.getValue() == 0.0)
				return 1.0;

			return 1.0/(1.0 + abs(curvature.getValue() * 0.5) * drivetrain->getTrackWidth().getValue());
		}

		void initialize() {
			startTime = currentTime();

			startDistance = drivetrain->getDistanceSinceReset();

			QVelocity adjustedSpeed = this->getSpeedMultiplier() * this->velocityProfile->getProfileConstraints().maxVelocity.getValue();

			this->velocityProfile->setDistance(this->distance);
			this->velocityProfile->setProfileConstraints({adjustedSpeed, velocityProfile->getProfileConstraints().maxAcceleration, velocityProfile->getProfileConstraints().maxJerk});

			this->velocityProfile->calculate();

			beforeBrakeMode = drivetrain->getBrakeMode();
			drivetrain->setBrakeMode(pros::MotorBrake::coast);
		}

		void update() override {
			QTime duration = currentTime() - startTime;

			drivetrainMutex.take();

			QAcceleration acceleration = velocityProfile->getAccelerationByTime(duration);
			QVelocity speed = velocityProfile->getVelocityByTime(duration);
			QLength targetDistance = velocityProfile->getDistanceByTime(duration);

			double turnPower;

			QLength currentDistance = (drivetrain->getDistanceSinceReset()-startDistance);

			distancePid->setTarget(targetDistance.getValue());

			double wheelVoltage = distancePid->update(currentDistance.getValue()) + velocityFeedforward * speed.Convert(inch/second) + accelerationFeedforward * acceleration.Convert(inch/second/second);

			// add curvature
			double leftCurvatureAdjustment = (2.0 + curvature.getValue() * drivetrain->getTrackWidth().getValue()) / 2.0;
			double rightCurvatureAdjustment = (2.0 - curvature.getValue() * drivetrain->getTrackWidth().getValue()) / 2.0;

			double leftVoltage = leftCurvatureAdjustment * wheelVoltage;
			double rightVoltage = rightCurvatureAdjustment * wheelVoltage;

			// integrate turnPid
			if (turnPid != nullptr) {
				Angle offset = targetDistance * curvature;

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

		bool isDone() override {
			return currentTime() - startTime > velocityProfile->getDuration();
		}

		~TankMotionProfiling() = default;
	};
} // namespace Pronounce
