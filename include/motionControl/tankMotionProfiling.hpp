#pragma once

#include "velocityProfile/velocityProfile.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "stateMachine/behavior.hpp"
#include "time/robotTime.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"

namespace Pronounce {
	class TankMotionProfiling : public Behavior {
	private:
		pros::Mutex& drivetrainMutex;

		TankDrivetrain* drivetrain;
		VelocityProfile* velocityProfile;

		QTime startTime = 0.0;

		ContinuousOdometry* odometry;

		Angle targetAngle = 0.0;

		PID* distancePid;
		PID* turnPid = nullptr;

		QCurvature curvature = 0.0;

		pros::motor_brake_mode_e_t beforeBrakeMode;

		QSpeed initialSpeed, endSpeed;
		QLength startDistance;

	public:
		TankMotionProfiling(std::string name, TankDrivetrain* drivetrain, VelocityProfile* velocityProfile, ContinuousOdometry* odometry, PID* distancePid, pros::Mutex& drivetrainMutex) : Behavior(name), drivetrain(drivetrain), velocityProfile(velocityProfile), odometry(odometry), drivetrainMutex(drivetrainMutex), distancePid(distancePid) {

		}

		TankMotionProfiling(std::string name, TankDrivetrain* drivetrain, ProfileConstraints profileConstraints, QLength distance, ContinuousOdometry* odometry, PID* distancePid, pros::Mutex& drivetrainMutex, QCurvature curvature, QSpeed initialSpeed = 0.0, QSpeed endSpeed = 0.0) : Behavior(name), initialSpeed(initialSpeed), endSpeed(endSpeed), drivetrain(drivetrain), odometry(odometry), drivetrainMutex(drivetrainMutex), distancePid(distancePid) {
			velocityProfile = new SinusoidalVelocityProfile(distance, profileConstraints, initialSpeed, endSpeed);
			velocityProfile->calculate(100);
			this->curvature = curvature;
		}

		TankMotionProfiling(std::string name, TankDrivetrain* drivetrain, ProfileConstraints profileConstraints, QLength distance, ContinuousOdometry* odometry, PID* distancePid, pros::Mutex& drivetrainMutex, QCurvature curvature, Angle targetAngle, PID* turnPid, QSpeed initialSpeed = 0.0, QSpeed endSpeed = 0.0) : Behavior(name), initialSpeed(initialSpeed), endSpeed(endSpeed), drivetrain(drivetrain), odometry(odometry), drivetrainMutex(drivetrainMutex), targetAngle(targetAngle), turnPid(turnPid), distancePid(distancePid) {
			velocityProfile = new SinusoidalVelocityProfile(distance, profileConstraints, initialSpeed, endSpeed);
			velocityProfile->calculate(100);
			this->curvature = curvature;
		}

		void initialize() {
			startTime = currentTime();

			// drivetrain->reset();
			startDistance = drivetrain->getDistanceSinceReset();

			aimingVisionSensor.set_led(COLOR_WHITE);

			drivetrain->tankSteerVelocity(0.0, 0.0);

			double curvatureAdjustment = std::min(((2.0 + curvature.getValue() * drivetrain->getTrackWidth().getValue()) / 2.0), ((2.0 - curvature.getValue() * drivetrain->getTrackWidth().getValue()) / 2.0));

			QSpeed adjustedSpeed = (1.0 - ((1-curvatureAdjustment)/2)) * velocityProfile->getProfileConstraints().maxVelocity;

			this->velocityProfile->setProfileConstraints({adjustedSpeed, velocityProfile->getProfileConstraints().maxAcceleration, velocityProfile->getProfileConstraints().maxJerk});

			this->velocityProfile->calculate(100);

			beforeBrakeMode = drivetrain->getBrakeMode();
			drivetrain->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
		}

		void update() {
			QTime duration = currentTime() - startTime;

			drivetrainMutex.take();

			QSpeed speed = velocityProfile->getVelocityByTime(duration);
			QLength distance = velocityProfile->getDistanceByTime(duration);

			double turnPower = 0;

			// calculate average wheel velocites
			QLength currentDistance = drivetrain->getDistanceSinceReset()-startDistance;

			distancePid->setTarget(distance.getValue() * signnum_c(this->velocityProfile->getDistance().getValue()));

			double wheelVoltage = distancePid->update(currentDistance.getValue());

			// add curvature
			double leftCurvatureAdjustment = (2.0 + curvature.getValue() * drivetrain->getTrackWidth().getValue()) / 2.0;
			double rightCurvatureAdjustment = (2.0 - curvature.getValue() * drivetrain->getTrackWidth().getValue()) / 2.0;

			double leftVoltage = leftCurvatureAdjustment * wheelVoltage;
			double rightVoltage = rightCurvatureAdjustment * wheelVoltage;

			// integrate turnPid
			if (turnPid != nullptr) {
				Angle offset = curvature * distance;

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

		void exit() {
			drivetrainMutex.take();
			
			drivetrain->setBrakeMode(beforeBrakeMode);

			drivetrain->skidSteerVelocity(0.0, 0.0);

			aimingVisionSensor.clear_led();

			drivetrainMutex.give();
		}

		bool isDone() {
			return currentTime() - startTime > velocityProfile->getDuration();
		}

		~TankMotionProfiling() {}
	};
} // namespace Pronounce
