#include "controller.hpp"

namespace Pronounce {

	Controller::Controller(pros::controller_id_e_t id) : pros::Controller(id) {
		this->odometry = nullptr;
	}

	Controller::Controller(pros::controller_id_e_t id, Odometry* odometry) : pros::Controller(id) {
		this->odometry = odometry;
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
		count++;
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
