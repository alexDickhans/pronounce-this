#pragma once

#include "stateMachine/behavior.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "velocityProfile/velocityProfile.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "utils/utils.hpp"

namespace Pronounce {
	class RamseteController : public Behavior {
	private:
		double b;
		double zeta;

		AbstractTankDrivetrain* drivetrain;
		ContinuousOdometry* odometry;

		VelocityProfile* velocityProfile;

		Pose2D desiredPose;
	public:
		RamseteController(AbstractTankDrivetrain* drivetrain, ContinuousOdometry* odometry, ProfileConstraints velocityProfile, Pose2D desiredPose, double b, double zeta) {
			this->drivetrain = drivetrain;
			this->odometry = odometry;
			this->velocityProfile = new SinusoidalVelocityProfile(10_in, velocityProfile);
			this->desiredPose = desiredPose;
			this->b = b;
			this->zeta = zeta;
		}

		void initialize() {
			velocityProfile->setDistance((this->odometry->getPose() - desiredPose).distance(Pose2D()));
			velocityProfile->calculate(10);
		}

		void update() {
			drivetrainMutex.take();

			QSpeed desiredSpeed = velocityProfile->getVelocityByDistance((this->odometry->getPose() - desiredPose).distance(Pose2D()));

			Pose2D error = desiredPose - odometry->getPose();

			std::cout << "DesiredSpeed: " << desiredSpeed.Convert(inch/second) << std::endl;

			// desiredSpeed = error.getY() / 1.0 / second;

			Angle desiredAngularVelocity = error.getAngle().getValue() * 0.01;

			double k = 2 * zeta * sqrt(pow(desiredAngularVelocity.getValue(), 2) + b * pow(desiredSpeed.getValue(), 2));

			QSpeed linearMotorVelocity = desiredSpeed.getValue() * -cos(error.getAngle()) + (k * error.getY().getValue());
			double angularMotorVelocity = desiredAngularVelocity.getValue() + (k * error.getAngle().getValue()) + ((b * desiredSpeed.getValue() * sin(error.getAngle()) * error.getY().getValue()) / error.getAngle().getValue());

			drivetrain->skidSteerVelocity(clamp(linearMotorVelocity.getValue(), (-10_in/second).getValue(), (10_in/second).getValue()), angularMotorVelocity);

			drivetrainMutex.give();
		}

		void exit() {
			drivetrain->skidSteerVelocity(0.0, 0.0);
		}

		bool isDone() {
			return false;
		}

		~RamseteController() {}
	};
} // namespace Pronounce
