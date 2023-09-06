#pragma once

#include "joystick.hpp"
#include "pros/misc.hpp"
#include "pros/misc.h"

namespace Pronounce {
	class RobotJoystick : public AbstractJoystick {
	private:
		pros::Controller* controller;
	public:
		explicit RobotJoystick(controller_id_e_t controllerId) : AbstractJoystick(controllerId) {
			controller = new pros::Controller((pros::controller_id_e_t) controllerId);
		}

        void update() override {
            for (int i = controller_digital_e_t::E_CONTROLLER_DIGITAL_L1; i != controller_digital_e_t::E_CONTROLLER_DIGITAL_A; i++) {
                if (this->get_digital_new_press(static_cast<controller_digital_e_t>(i))) {
					std::cout << "HIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << i << this->callbacks[static_cast<controller_digital_e_t>(i)].size() << std::endl << std::endl << std::endl;
                    std::for_each(this->callbacks[static_cast<controller_digital_e_t>(i)].begin(), this->callbacks[static_cast<controller_digital_e_t>(i)].end(), [=](const VoidCallback& callback) -> void {callback();});
                }
            }
        }

		std::int32_t is_connected() override {
			return controller->is_connected();
		}

		std::int32_t get_analog(controller_analog_e_t channel) override { return controller->get_analog((pros::controller_analog_e_t) channel); }

		std::int32_t get_digital(controller_digital_e_t channel) override { return controller->get_digital((pros::controller_digital_e_t) channel); }

		std::int32_t get_digital_new_press(controller_digital_e_t channel) override { return controller->get_digital_new_press((pros::controller_digital_e_t) channel); }

		~RobotJoystick() {
            delete controller;
        };
	};
} // namespace Pronounce
