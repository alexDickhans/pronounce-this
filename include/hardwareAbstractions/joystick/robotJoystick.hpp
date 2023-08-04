#pragma once

#include "joystick.hpp"
#include "pros/misc.hpp"
#include "pros/misc.h"

namespace Pronounce {
	class RobotJoystick : public AbstractJoystick {
	private:
		pros::Controller* controller;
	public:
		RobotJoystick(controller_id_e_t controllerId) : AbstractJoystick(controllerId) {
			controller = new pros::Controller((pros::controller_id_e_t) controllerId);
		}

		virtual std::int32_t is_connected() {
			return controller->is_connected();
		}

		virtual std::int32_t get_analog(controller_analog_e_t channel) { return controller->get_analog((pros::controller_analog_e_t) channel); }

		virtual std::int32_t get_digital(controller_digital_e_t channel) { return controller->get_digital((pros::controller_digital_e_t) channel); }

		virtual std::int32_t get_digital_new_press(controller_digital_e_t channel) { return controller->get_digital_new_press((pros::controller_digital_e_t) channel); }

		~RobotJoystick() {}
	};
} // namespace Pronounce
