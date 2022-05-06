#pragma once

#include "purePursuit.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include <iostream>
#include "api.h"

namespace Pronounce {
	class TankPurePursuit : public PurePursuit {
	private:
		AbstractTankDrivetrain* drivetrain;

		double speed = 100;
		bool inverted = false;

		bool orientationControl = false;
		PID* turnPid;

		double lastUpdateTime = 0;

		bool useVoltage = false;
		bool ignoreAcceleration = false;
	public:
		TankPurePursuit(AbstractTankDrivetrain* drivetrain);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, double lookaheadDistance);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, Odometry* odometry, double lookaheadDistance);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, Odometry* odometry, PID* turnPid, double lookaheadDistance);

		void updateDrivetrain();

		void stop();

		bool getUseVoltage() {
			return useVoltage;
		}

		void setUseVoltage(bool useVoltage) {
			this->useVoltage = useVoltage;
		}

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
			this->turnPid->reset();
		}

		bool getOrientationControl() {
			return orientationControl;
		}

		void setOrientationControl(bool orientationControl) {
			this->orientationControl = orientationControl;
		}

		bool isDone(double maxDistance) {
			return maxDistance > this->getPaths().at(this->getCurrentPathIndex()).distanceFromEnd(Point(this->getOdometry()->getPosition()->getX(), this->getOdometry()->getPosition()->getY()));
		}

		bool isDoneOrientation(double maxDifference) {
			return maxDifference > turnPid->getError();
		}

		~TankPurePursuit();
	};
} // namespace Pronounce
