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
		double speed = linearController->update(imu->get_roll());

		printf("speed: %f, angle: %f. \n", speed, imu->get_roll());

		// Calculate the turing force with the orientation PID
		this->orientationController->setPosition(toRadians(this->imu->get_heading()));
		double turn = this->orientationController->update();

		printf("Speed: %f, Turn: %f\n", speed, turn);

		// Send the calculated values to the drivetrain
		this->drivetrain->skidSteerVelocity(speed, turn);
	}

	Balance::~Balance() {
	}
} // namespace Pronounce
