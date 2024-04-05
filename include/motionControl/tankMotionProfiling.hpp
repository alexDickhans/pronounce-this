#pragma once

#include <utility>

#include "velocityProfile/velocityProfile.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class TankMotionProfiling : public Behavior {
	private:

		TankDrivetrain& drivetrain;
		std::shared_ptr<VelocityProfile> velocityProfile;

		QTime startTime = 0.0;

		std::function<Angle()> angleFunction;

		Angle targetAngle = 0.0;

		FeedbackController* distancePid;
		PID* turnPid = nullptr;

		QLength distance;
		QCurvature curvature;

		QLength startDistance;

		std::function<double(QVelocity, QAcceleration)> feedforwardFunction;

	public:
		TankMotionProfiling(std::string name, TankDrivetrain &drivetrain,
		                    std::shared_ptr<TrapezoidalVelocityProfile> velocityProfile,
		                    std::function<Angle()> angleFunction, FeedbackController *distancePid) :
				Behavior(std::move(name)),
				drivetrain(drivetrain),
				velocityProfile(velocityProfile),
				angleFunction(std::move(angleFunction)),
				distancePid(distancePid) {

		}

		TankMotionProfiling(
				std::string name,
				TankDrivetrain& drivetrain,
				ProfileConstraints profileConstraints,
				QLength distance,
				std::function<Angle()> angleFunction,
				FeedbackController* distancePid,
				QCurvature curvature,
				std::function<double(QVelocity, QAcceleration)> feedforwardFunction,
				QVelocity initialSpeed = 0.0,
				QVelocity endSpeed = 0.0) :
					Behavior(std::move(name)),
					drivetrain(drivetrain),
					angleFunction(std::move(angleFunction)),
					distancePid(distancePid),
					feedforwardFunction(std::move(feedforwardFunction)) {
			velocityProfile = std::make_shared<TrapezoidalVelocityProfile>(fabs(distance.getValue()), profileConstraints, initialSpeed, endSpeed);
			velocityProfile->calculate();
			this->distance = distance;
			this->curvature = curvature;
		}

		TankMotionProfiling(
				std::string name,
				TankDrivetrain& drivetrain,
				ProfileConstraints profileConstraints,
				QLength distance,
				std::function<Angle()> angleFunction,
				PID* distancePid,
				QCurvature curvature,
				std::function<double(QVelocity, QAcceleration)> feedforwardFunction,
				Angle targetAngle,
				PID* turnPid,
				QVelocity initialSpeed = 0.0,
				QVelocity endSpeed = 0.0) :
					Behavior(std::move(name)),
					drivetrain(drivetrain),
					angleFunction(std::move(angleFunction)),
					targetAngle(targetAngle),
					turnPid(turnPid),
					distancePid(distancePid),
					feedforwardFunction(std::move(feedforwardFunction)) {
			velocityProfile = std::make_shared<TrapezoidalVelocityProfile>(distance, profileConstraints, initialSpeed, endSpeed);
			velocityProfile->calculate();
			this->distance = distance;
			this->curvature = curvature;
		}

		double getSpeedMultiplier() {

			if (curvature.getValue() == 0.0)
				return 1.0;

			return 1.0/(1.0 + abs(curvature.getValue() * 0.5) * drivetrain.getTrackWidth().getValue());
		}

		void initialize() override {
			startTime = pros::millis() * 1_ms;

			startDistance = drivetrain.getDistanceSinceReset();

			QVelocity adjustedSpeed = this->getSpeedMultiplier() * this->velocityProfile->getProfileConstraints().maxVelocity.getValue();

			this->velocityProfile->setDistance(this->distance);
			this->velocityProfile->setProfileConstraints({adjustedSpeed, velocityProfile->getProfileConstraints().maxAcceleration, velocityProfile->getProfileConstraints().maxJerk});

			this->velocityProfile->calculate();
		}

		void update() override {
			QTime duration = (pros::millis() * 1_ms) - startTime;

			drivetrainMutex.lock();

			QAcceleration acceleration = velocityProfile->getAccelerationByTime(duration);
			QVelocity speed = velocityProfile->getVelocityByTime(duration);
			QLength targetDistance = velocityProfile->getDistanceByTime(duration);

			double turnPower;

			QLength currentDistance = (drivetrain.getDistanceSinceReset()-startDistance);

			Log(string_format("targetDistance: %f, currentDistance: %f", targetDistance.Convert(inch), currentDistance.Convert(inch)));

			distancePid->setTarget(targetDistance.getValue());

			double wheelVoltage = distancePid->update(currentDistance.getValue()) + feedforwardFunction(speed, acceleration);

			// add curvature
			double leftCurvatureAdjustment = (2.0 + curvature.getValue() * drivetrain.getTrackWidth().getValue()) / 2.0;
			double rightCurvatureAdjustment = (2.0 - curvature.getValue() * drivetrain.getTrackWidth().getValue()) / 2.0;

			double leftVoltage = leftCurvatureAdjustment * wheelVoltage;
			double rightVoltage = rightCurvatureAdjustment * wheelVoltage;

			// integrate turnPid
			if (turnPid != nullptr) {
				Angle offset = targetDistance * curvature;

				Angle targetAngleWithOffset = targetAngle + offset;

				turnPid->setTarget(targetAngleWithOffset.getValue());
				
				turnPower = turnPid->update(angleFunction().Convert(radian));

				leftVoltage += turnPower;
				rightVoltage -= turnPower;
			}

			// send to motors
			drivetrain.tankSteerVoltage(leftVoltage, rightVoltage);

			drivetrainMutex.give();
		}

		void exit() override {
			drivetrainMutex.take();

			drivetrain.tankSteerVoltage(0.0, 0.0);

			drivetrainMutex.give();
		}

		bool isDone() override {
			return (pros::millis() * 1_ms) - startTime > velocityProfile->getDuration();
		}

		~TankMotionProfiling() = default;
	};
} // namespace Pronounce
