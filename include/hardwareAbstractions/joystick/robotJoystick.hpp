#pragma once

#include "joystick.hpp"
#include "pros/misc.hpp"
#include "pros/misc.h"

namespace Pronounce {
	class RobotJoystick : public AbstractJoystick {
	private:
		pros::Controller* controller;
        pros::Task* task;
	public:
		explicit RobotJoystick(controller_id_e_t controllerId) : AbstractJoystick(controllerId) {
			controller = new pros::Controller((pros::controller_id_e_t) controllerId);
            task = new pros::Task([=]() -> void {update(); pros::Task::delay(10);});
		}

        void update() override {
            for (int i = 6; i <= 17; i++) {
                if (controller->get_digital_new_press((pros::controller_digital_e_t) i)) {
                    std::for_each(this->callbacks[(controller_digital_e_t) i].begin(), this->callbacks[(controller_digital_e_t) i].end(), [=](const VoidCallback& callback) -> void {callback();});
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
