#pragma once

#include "purePursuit.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include <iostream>

// TODO: Clean this up and add doc strings and comments

namespace Pronounce {
	class TankPurePursuit : public PurePursuit {
	private:
		AbstractTankDrivetrain* drivetrain;

		QLength speed = 100.0;
		bool inverted = false;

		bool orientationControl = false;
		PID* turnPid;

		bool useVoltage = false;
		bool ignoreAcceleration = false;
	public:
		TankPurePursuit(AbstractTankDrivetrain* drivetrain);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, double lookaheadDistance);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, ContinuousOdometry* odometry, double lookaheadDistance);
		TankPurePursuit(AbstractTankDrivetrain* drivetrain, ContinuousOdometry* odometry, PID* turnPid, double lookaheadDistance);

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

		QLength getSpeed() {
			return speed;
		}

		void setSpeed(QLength speed) {
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

		bool isDone(QLength maxDistance) {
			return maxDistance > this->getPath().distanceFromEnd(Point(this->getOdometry()->getPosition()->getX(), this->getOdometry()->getPosition()->getY()));
		}

		bool isDoneOrientation(double maxDifference) {
			return maxDifference > turnPid->getError();
		}

		~TankPurePursuit();
	};
} // namespace Pronounce
