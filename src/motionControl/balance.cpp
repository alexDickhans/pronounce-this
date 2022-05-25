#include "balance.hpp"

namespace Pronounce {
	Balance::Balance() : Balance(nullptr, nullptr) {
	}

	Balance::Balance(AbstractTankDrivetrain* drivetrain) : Balance(drivetrain, nullptr) {
	}

	Balance::Balance(AbstractTankDrivetrain* drivetrain, pros::Imu* imu) : Balance(drivetrain, imu, new BangBang(), new PID()) {
	}

	Balance::Balance(AbstractTankDrivetrain* drivetrain, pros::Imu* imu, BangBang* linearController, PID* orientationController) {
		this->drivetrain = drivetrain;
		this->imu = imu;
		this->linearController = linearController;
		this->orientationController = orientationController;
		orientationController->setTurnPid(true);
	}

	void Balance::update() {
		if (imu == nullptr || drivetrain == nullptr || !enabled)
			return;

		// Set the speed with the bang bang controller
		double speed = linearController->update(imu->get_roll()+7);

		// Calculate the turing force with the orientation PID
		double turn = this->orientationController->update(toRadians(this->imu->get_heading()));

		// Send the calculated values to the drivetrain
		this->drivetrain->skidSteerVelocity(speed, turn);
	}

	Balance::~Balance() {
	}
} // namespace Pronounce
