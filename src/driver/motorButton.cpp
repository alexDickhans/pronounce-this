#include "motorButton.hpp"

namespace Pronounce {
	MotorButton::MotorButton(pros::Controller* controller, pros::Motor* motor,
		pros::controller_digital_e_t positiveButton,
		pros::controller_digital_e_t negativeButton,
		int positiveAuthority,
		int neutralAuthority,
		int negativeAuthority,
		int min,
		int max) : Button(controller, positiveButton, negativeButton) {
		this->motor = motor;
		this->positiveAuthority = positiveAuthority;
		this->neutralAuthority = neutralAuthority;
		this->negativeAuthority = negativeAuthority;
		this->min = min;
		this->max = max;
	}

	void MotorButton::updateActuator() {
		if (!this->getEnabled()) {
			this->motor->move_velocity(0.0);
			return;
		}

		if (this->getAutonomous() && this->autonomousPosition) {
			if (this->goToHeight) {
				// Use autonomous authority to calculate the angle that the lift needs to be at. 
				double angle = asin((this->autonomousAuthority - this->height) / this->length) * this->gearRatio;

				// Set the lift to the angle.
				this->motor->move_absolute(angle, abs(this->positiveAuthority));
			}
			else {
				this->motor->move_absolute(this->autonomousAuthority, abs(this->positiveAuthority));
			}
			return;
		}

		switch (this->getButtonStatus()) {
		case NEGATIVE:
			if (goToImmediately) {
				this->motor->move_absolute(min, negativeAuthority);
			}
			else if (this->motor->get_position() > this->min && this->min < this->max) {
				this->motor->move(negativeAuthority);
			}
			else if (this->min >= this->max) {
				this->motor->move(negativeAuthority);
			}
			else {
				this->motor->move_velocity(neutralAuthority);
			}
			break;
		case POSITIVE:
			if (goToImmediately) {
				this->motor->move_absolute(max, positiveAuthority);
			}
			else if (this->motor->get_position() < this->max && this->min < this->max) {
				this->motor->move(positiveAuthority);
			}
			else if (this->min >= this->max) {
				this->motor->move(positiveAuthority);
			}
			else {
				this->motor->move_velocity(neutralAuthority);
			}
			break;
		case NEUTRAL:
		default:
			if (this->getSingleToggle() && goToImmediately) {
				this->motor->move_absolute(min, neutralAuthority);
			}
			if (goToImmediately)
				return;
			this->motor->move_velocity(neutralAuthority);
			break;
		}

		if (!this->getDejam()) {
			return;
		}

		double speed = this->motor->get_actual_velocity();
		bool jammed = this->jammed;

		// set jammed to true if the speed of the motor indicates that it is jammed
		if (speed < dejamSpeed && this->getButtonStatus() != NEUTRAL) {
			printf("Jammed\n");
			this->jammed = true;
		}
		else {
			this->jammed = false;
			this->dejamStartTime = pros::millis();
		}

		// Debounce to start the time
		if (jammed && !this->jammed) {
			this->dejamStartTime = pros::millis();
		}

		// printf("Dejam time: %f\n", pros::millis() - this->dejamStartTime);

		// If the timer is greater than the jam time, then dejam the motor
		if (pros::millis() - this->dejamStartTime > dejamTime && jammed && pros::millis() - this->dejamStartTime < dejamTime + dejamDuration || ((this->jammed && pros::millis() - this->dejamStartTime) % 10000 > 5000)) {
			printf("Dejam started\n");
			this->motor->move_velocity(dejamAuthority);
		}
	}

	MotorButton::~MotorButton() {
	}
} // namespace Pronounce
