#include "controller.hpp"

namespace Pronounce {

	Controller::Controller(pros::controller_id_e_t id) : pros::Controller(id) {
		this->robotPosition = nullptr;
	}

	Controller::Controller(pros::controller_id_e_t id, Position* robotPosition) : pros::Controller(id) {
		this->robotPosition = robotPosition;
	}

	double Controller::getMagnitude(int joystick) {
		double joystickX;
		double joystickY;

		switch (joystick) {
		case PRONOUNCE_CONTROLLER_RIGHT:
			joystickX = this->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
			joystickY = this->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		case PRONOUNCE_CONTROLLER_LEFT:
		default:
			joystickX = this->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
			joystickY = this->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

		}

		double magnitude = sqrt(pow(joystickX, 2) + pow(joystickY, 2)); // √leftX^2 + leftY^2
		return magnitude;
	}

	double Controller::getTheta(int joystick) {
		double joystickX;
		double joystickY;

		switch (joystick) {
		case PRONOUNCE_CONTROLLER_RIGHT:
			joystickX = this->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
			joystickY = this->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		case PRONOUNCE_CONTROLLER_LEFT:
		default:
			joystickX = this->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
			joystickY = this->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		}

		double angle = atan2(joystickY, joystickX); // atan2(leftY, leftX)
		return angle;
	}

	Vector Controller::getVector(int joystick) {
		return Vector(this->getMagnitude(joystick), this->getTheta(joystick));
	}

	void Controller::render() {
		if (!continueRendering) {
			return;
		}

		this->clear();
		pros::Task::delay(120);


		// During disabled
		if (pros::competition::is_disabled() && pros::competition::is_connected()) {
			this->print(0, 0, "Robot disabled");
		}
		else if (pros::competition::is_autonomous() && pros::competition::is_connected()) {
			this->print(0, 0, "Auton Period");
		}
		else {
			this->print(0, 0, "DT: %fC", this->drivetrain->getTemp());
			pros::Task::delay(120);
			this->print(1, 0, "DT: %f%", this->drivetrain->getSpeed());
			pros::Task::delay(120);
		}
		
		this->print(2, 0, "Battery: %f%", pros::battery::get_capacity());
		pros::Task::delay(120);

		if (pros::battery::get_capacity() < 20.0) {
			this->rumble(".");
			pros::Task::delay(120);
		}

		pros::Task::delay(200);
	}

	void Controller::renderFunc() {
		while (true) {
			this->render();

			// Prevent wasted resources
			pros::Task::delay(100);
		}
	}

	void Controller::startRenderThread() {
		//this->renderTask = &pros::Task(pros::task_fn_t {this->renderFunc()}, "renderTask");
	}

	Controller::~Controller() {

	}
}
