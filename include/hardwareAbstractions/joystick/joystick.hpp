#pragma once

namespace Pronounce {

	typedef enum {
		E_CONTROLLER_ANALOG_LEFT_X = 0,
		E_CONTROLLER_ANALOG_LEFT_Y,
		E_CONTROLLER_ANALOG_RIGHT_X,
		E_CONTROLLER_ANALOG_RIGHT_Y
	} controller_analog_e_t;

	typedef enum {
		E_CONTROLLER_DIGITAL_L1 = 6,
		E_CONTROLLER_DIGITAL_L2,
		E_CONTROLLER_DIGITAL_R1,
		E_CONTROLLER_DIGITAL_R2,
		E_CONTROLLER_DIGITAL_UP,
		E_CONTROLLER_DIGITAL_DOWN,
		E_CONTROLLER_DIGITAL_LEFT,
		E_CONTROLLER_DIGITAL_RIGHT,
		E_CONTROLLER_DIGITAL_X,
		E_CONTROLLER_DIGITAL_B,
		E_CONTROLLER_DIGITAL_Y,
		E_CONTROLLER_DIGITAL_A
	} controller_digital_e_t;

	typedef enum { E_CONTROLLER_MASTER = 0, E_CONTROLLER_PARTNER } controller_id_e_t;



	class AbstractJoystick {
	private:
		/* data */
	public:
		AbstractJoystick(controller_id_e_t controllerId) {}

		virtual std::int32_t is_connected() {
			return false;
		}

		virtual std::int32_t get_analog(controller_analog_e_t channel) { return 0; }

		virtual std::int32_t get_digital(controller_digital_e_t channel) { return 0; }

		virtual std::int32_t get_digital_new_press(controller_digital_e_t channel) { return 0; }

		~AbstractJoystick() {}
	};
} // namespace Pronounce