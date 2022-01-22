#pragma once

#include "purePursuit.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include <iostream>

namespace Pronounce {
	class TankPurePursuit : public PurePursuit {
	private:
		AbstractTankDrivetrain* drivetrain;

		double speed = 100;
		bool inverted = false;

		bool orientationControl = false;
		PID* turnPid;
	public:
		TankPurePursuit(AbstractTankDrivetrain* drivetrain);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, double lookaheadDistance);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, Odometry* odometry, double lookaheadDistance);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, Odometry* odometry, PID* turnPid, double lookaheadDistance);

		void updateDrivetrain();

		void stop();

		AbstractTankDrivetrain* getDrivetrain() {
			return drivetrain;
		}

		void setDrivetrain(AbstractTankDrivetrain* drivetrain) {
			this->drivetrain = drivetrain;
		}

		double getSpeed() {
			return speed;
		}

		void setSpeed(double speed) {
			this->speed = speed;
		}

		bool getInverted() {
			return inverted;
		}

		void setInverted(bool inverted) {
			this->inverted = inverted;
		}

		PID* getTurnPid() {
			return turnPid;
		}

		void setTurnPid(PID* turnPid) {
			this->turnPid = turnPid;
			this->turnPid->setTurnPid(true);
		}

		double getTargetOrientation() {
			return this->turnPid->getTarget();
		}

		void setTargetOrientation(double orientation) {
			this->turnPid->setTarget(orientation);
		}

		bool getOrientationControl() {
			return orientationControl;
		}

		void setOrientationControl(bool orientationControl) {
			this->orientationControl = orientationControl;
		}

		bool isDone(double maxDistance) {
			return maxDistance > this->getOdometry()->getPosition()->distance(this->getPath(this->getCurrentPathIndex()).getPoint(this->getPath(this->getCurrentPathIndex()).getPath().size() - 1));
		}

		bool isDoneOrientation(double maxDifference) {
			return maxDifference > angleDifference(this->getOdometry()->getPosition()->getTheta(), this->getTurnPid()->getTarget()) && this->turnPid->getDerivitive() < maxDifference/5.0;
		}

		~TankPurePursuit();
	};
} // namespace Pronounce
