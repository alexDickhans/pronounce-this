#pragma once

#include "feedbackControllers/bangBang.hpp"	
#include "feedbackControllers/pid.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "api.h"

namespace Pronounce {
	class Balance
	{
	private:
		BangBang* linearController;
		PID* orientationController;

		bool enabled = false;

		AbstractTankDrivetrain* drivetrain;
		pros::Imu* imu;
	public:
		Balance();
		Balance(AbstractTankDrivetrain* drivetrain);
		Balance(AbstractTankDrivetrain* drivetrain, pros::Imu* imu);
		Balance(AbstractTankDrivetrain* drivetrain, pros::Imu* imu, BangBang* linearController, PID* orientationController);

		void update();

		void balance(double waitTime) {
			while(!this->isBalanced()) {
				this->update();
				pros::Task::delay(waitTime);
			}
		}

		bool isBalanced() {
			return linearController->isInRange();
		}

		BangBang* getLinearController() {
			return this->linearController;
		}

		void setLinearController(BangBang* linearController) {
			this->linearController = linearController;
		}

		PID* getOrientationController() {
			return this->orientationController;
		}

		void setOrientationController(PID* orientationController) {
			this->orientationController = orientationController;
		}

		bool isEnabled() {
			return this->enabled;
		}

		void setEnabled(bool enabled) {
			this->enabled = enabled;
		}

		AbstractTankDrivetrain* getDrivetrain() {
			return this->drivetrain;
		}

		void setDrivetrain(AbstractTankDrivetrain* drivetrain) {
			this->drivetrain = drivetrain;
		}

		pros::Imu* getImu() {
			return this->imu;
		}

		void setImu(pros::Imu* imu) {
			this->imu = imu;
		}


		~Balance();
	};
} // namespace Pronounce
