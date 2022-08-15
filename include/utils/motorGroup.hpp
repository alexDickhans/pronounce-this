#pragma once

#include <vector>
#include "api.h"

// TODO: Add docstring

namespace Pronounce {

	/**
	 * @brief Group motors together to combine their outputs
	 *
	 * @authors Alex Dickhans(alexDickahns)
	 */
	class MotorGroup {
	private:
		/**
		 * @brief List of motor pointers
		 *
		 */
		std::vector<pros::Motor*> motors;
	public:
		/**
		 * @brief Construct a new Motor Group object with no motors
		 *
		 */
		MotorGroup();

		/**
		 * @brief Construct a new Motor Group object with motors
		 *
		 * @param motors List of motors
		 */
		MotorGroup(std::vector<pros::Motor*> motors);

		/**
		 * Sets the voltage for the motor from -128 to 127.
		 *
		 * This is designed to map easily to the input from the controller's analog
		 * stick for simple opcontrol use. The actual behavior of the motor is
		 * analogous to use of pros::Motor::move(), or motorSet from the PROS 2 API.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \param voltage
		 *        The new motor voltage from -127 to 127
		 *
		 * \return 1 if the operation was successful or PROS_ERR if the operation
		 * failed, setting errno.
		 */
		void operator=(std::int32_t power) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move_voltage(power);
			}
		}

		/**
		 * @brief Add a motor to the list
		 *
		 * @param motor Pointer to the new motor object to add
		 */
		void addMotor(pros::Motor* motor) {
			this->motors.emplace_back(motor);
		}

		/**
		 * Sets the voltage for the motor from -127 to 127.
		 *
		 * This is designed to map easily to the input from the controller's analog
		 * stick for simple opcontrol use. The actual behavior of the motor is
		 * analogous to use of motor_move(), or motorSet() from the PROS 2 API.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \param voltage
		 *        The new motor voltage from -127 to 127
		 *
		 * \return 1 if the operation was successful or PROS_ERR if the operation
		 * failed, setting errno.
		 */
		void move(double power) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move(power);
			}
		}
		
		/**
		 * Sets the target absolute position for the motor to move to.
		 *
		 * This movement is relative to the position of the motor when initialized or
		 * the position when it was most recently reset with
		 * pros::Motor::set_zero_position().
		 *
		 * \note This function simply sets the target for the motor, it does not block
		 * program execution until the movement finishes.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \param position
		 *        The absolute position to move to in the motor's encoder units
		 * \param velocity
		 *        The maximum allowable velocity for the movement in RPM
		 *
		 * \return 1 if the operation was successful or PROS_ERR if the operation
		 * failed, setting errno.
		 */
		void move_absolute(double position, std::int32_t velocity) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move_absolute(position, velocity);
			}
		}

		/**
		 * Sets the relative target position for the motor to move to.
		 *
		 * This movement is relative to the current position of the motor as given in
		 * pros::Motor::motor_get_position(). Providing 10.0 as the position parameter
		 * would result in the motor moving clockwise 10 units, no matter what the
		 * current position is.
		 *
		 * \note This function simply sets the target for the motor, it does not block
		 * program execution until the movement finishes.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \param position
		 *        The relative position to move to in the motor's encoder units
		 * \param velocity
		 *        The maximum allowable velocity for the movement in RPM
		 *
		 * \return 1 if the operation was successful or PROS_ERR if the operation
		 * failed, setting errno.
		 */
		void move_relative(double position, std::int32_t velocity) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move_relative(position, velocity);
			}
		}

		/**
		 * Sets the velocity for the motor.
		 *
		 * This velocity corresponds to different actual speeds depending on the
		 * gearset used for the motor. This results in a range of +-100 for
		 * E_MOTOR_GEARSET_36, +-200 for E_MOTOR_GEARSET_18, and +-600 for
		 * E_MOTOR_GEARSET_6. The velocity is held with PID to ensure consistent
		 * speed, as opposed to setting the motor's voltage.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \param velocity
		 *        The new motor velocity from -+-100, +-200, or +-600 depending on the
		 *        motor's gearset
		 *
		 * \return 1 if the operation was successful or PROS_ERR if the operation
		 * failed, setting errno.
		 */
		void move_velocity(double velocity) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move_velocity(velocity);
			}
		}

		/**
		 * Sets the output voltage for the motor from -12000 to 12000 in millivolts.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \param port
		 *        The V5 port number from 1-21
		 * \param voltage
		 *        The new voltage value from -12000 to 12000
		 *
		 * \return 1 if the operation was successful or PROS_ERR if the operation
		 * failed, setting errno.
		 */
		void move_voltage(std::int32_t voltage) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move_voltage(voltage);
			}
		}

		/**
		 * Stops the motor using the currently configured brake mode.
		 *
		 * This function sets motor velocity to zero, which will cause it to act
		 * according to the set brake mode. If brake mode is set to MOTOR_BRAKE_HOLD,
		 * this function may behave differently than calling move_absolute(0)
		 * or move_relative(0).
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return 1 if the operation was successful or PROS_ERR if the operation
		 * failed, setting errno.
		 */
		void brake() {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->brake();
			}
		}

		/**
		 * Changes the output velocity for a profiled movement (motor_move_absolute()
		 * or motor_move_relative()). This will have no effect if the motor is not
		 * following a profiled movement.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \param velocity
		 *        The new motor velocity from +-100, +-200, or +-600 depending on the
		 *        motor's gearset
		 *
		 * \return 1 if the operation was successful or PROS_ERR if the operation
		 * failed, setting errno.
		 */
		void modify_profiled_velocity(double voltage) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->modify_profiled_velocity(voltage);
			}
		}

		/**
		 * Gets the target position set for the motor by the user.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return The target position in its encoder units or PROS_ERR_F if the
		 * operation failed, setting errno.
		 */
		double get_target_position() {
			double targetPosition = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				targetPosition += motor->get_target_position();
			}
			return targetPosition / motors.size();
		}

		/**
		 * Gets the velocity commanded to the motor by the user.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return The commanded motor velocity from +-100, +-200, or +-600, or
		 * PROS_ERR if the operation failed, setting errno.
		 */	
		double get_target_velocity() {
			double targetVelocity = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				targetVelocity += motor->get_target_velocity();
			}
			return targetVelocity / motors.size();
		}

		/**
		 * Gets the actual velocity of the motor.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return The motor's actual velocity in RPM or PROS_ERR_F if the operation
		 * failed, setting errno.
		 */
		double get_actual_velocity() {
			double actualVelocity = 0;
			for (int i = 0; i < motors.size(); i++) {
				actualVelocity += this->motors.at(i)->get_actual_velocity();
			}
			return actualVelocity / motors.size();
		}

		/**
		 * Gets the current drawn by the motor in mA.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return The motor's current in mA or PROS_ERR if the operation failed,
		 * setting errno.
		 */
		double get_current_draw() {
			double currentDraw = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				currentDraw += motor->get_current_draw();
			}
			return currentDraw / motors.size();
		}

		/**
		 * Gets the direction of movement for the motor.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return 1 for moving in the positive direction, -1 for moving in the
		 * negative direction, and PROS_ERR if the operation failed, setting errno.
		 */
		std::int32_t get_direction() {
			std::int32_t direction = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				direction += motor->get_direction();
			}
			return direction / motors.size();
		}

		/**
		 * Gets the efficiency of the motor in percent.
		 *
		 * An efficiency of 100% means that the motor is moving electrically while
		 * drawing no electrical power, and an efficiency of 0% means that the motor
		 * is drawing power but not moving.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return The motor's efficiency in percent or PROS_ERR_F if the operation
		 * failed, setting errno.
		 */
		double get_efficiency() {
			double efficiency = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				efficiency += motor->get_efficiency();
			}
			return efficiency / motors.size();
		}

		/**
		 * Checks if the motor is drawing over its current limit.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return 1 if the motor's current limit is being exceeded and 0 if the
		 * current limit is not exceeded, or PROS_ERR if the operation failed, setting
		 * errno.
		 */
		double is_over_current() {
			double overCurrent = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				overCurrent += motor->is_over_current();
			}
			return overCurrent / motors.size();
		}

		/**
		 * Checks if the motor is stopped.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \note Although this function forwards data from the motor, the motor
		 * presently does not provide any value. This function returns PROS_ERR with
		 * errno set to ENOSYS.
		 *
		 * \return 1 if the motor is not moving, 0 if the motor is moving, or PROS_ERR
		 * if the operation failed, setting errno
		 */
		double is_stopped() {
			double stopped = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				stopped += motor->is_stopped();
			}
			return stopped / motors.size();
		}

		/**
		 * Checks if the motor is at its zero position.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \note Although this function forwards data from the motor, the motor
		 * presently does not provide any value. This function returns PROS_ERR with
		 * errno set to ENOSYS.
		 *
		 * \return 1 if the motor is at zero absolute position, 0 if the motor has
		 * moved from its absolute zero, or PROS_ERR if the operation failed, setting
		 * errno
		 */
		double get_zero_position_flag() {
			double zeroPosition = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				zeroPosition += motor->get_zero_position_flag();
			}
			return zeroPosition / motors.size();
		}

		/**
		 * Gets the power drawn by the motor in Watts.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return The motor's power draw in Watts or PROS_ERR_F if the operation
		 * failed, setting errno.
		 */
		double get_power() {
			double stopped = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				stopped += motor->get_power();
			}
			return stopped / motors.size();
		}
	
		/**
		 * Gets the temperature of the motor in degrees Celsius.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return The motor's temperature in degrees Celsius or PROS_ERR_F if the
		 * operation failed, setting errno.
		 */
		double get_temperature() {
			double stopped = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				stopped += motor->get_temperature();
			}
			return stopped / motors.size();
		}

		/**
		 * Gets the torque generated by the motor in Newton Meters (Nm).
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return The motor's torque in Nm or PROS_ERR_F if the operation failed,
		 * setting errno.
		 */
		double get_torque() {
			double stopped = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				stopped += motor->get_torque();
			}
			return stopped / motors.size();
		}

		/**
		 * Gets the voltage delivered to the motor in millivolts.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \return The motor's voltage in mV or PROS_ERR_F if the operation failed,
		 * setting errno.
		 */
		double get_voltage() {
			double stopped = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				stopped += motor->get_voltage();
			}
			return stopped / motors.size();
		}

		/**
		 * Sets one of motor_brake_mode_e_t to the motor.
		 *
		 * This function uses the following values of errno when an error state is
		 * reached:
		 * ENODEV - The port cannot be configured as a motor
		 *
		 * \param mode
		 *        The motor_brake_mode_e_t to set for the motor
		 *
		 * \return 1 if the operation was successful or PROS_ERR if the operation
		 * failed, setting errno.
		 */
		void set_brake_mode(pros::motor_brake_mode_e_t mode) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->set_brake_mode(mode);
			}
		}

		~MotorGroup();
	};
} // namespace Pronounce
