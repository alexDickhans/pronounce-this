#pragma once

#include "api.h"
#include "controller.hpp"
#include "button.hpp"

namespace Pronounce {

    class MotorButton : public Button {
    private:
        bool goToImmediately = false;

        bool autonomousPosition = false;

		bool dejam = false;
		double dejamTime = 0;
		double dejamSpeed = 0;
		double dejamAuthority = 0;
		double dejamStartTime = 0;
		double jamSpeed = 50;

		bool jammed = false;

        int positiveAuthority = 0;
        int neutralAuthority = 0;
        int negativeAuthority = 0;

        int autonomousAuthority = 0;

		bool goToHeight = false;
		double height;
		double length;

		double gearRatio;

        int min = 0;
        int max = 0;
        pros::Motor* motor;

    public:
        MotorButton();
        MotorButton(pros::Controller* controller, pros::Motor* motor,
        pros::controller_digital_e_t positiveButton = pros::E_CONTROLLER_DIGITAL_L1,
        pros::controller_digital_e_t negativeButton = pros::E_CONTROLLER_DIGITAL_L2,
        int positiveAuthority = 0,
        int neutralAuthority = 0,
        int negativeAuthority = 0,
        int min = 0,
        int max = 0);
        
        void updateActuator();

		void resetPosition(double currentPosition) {
			this->motor->set_zero_position(-currentPosition);
		}

        bool getGoToImmediately() {
            return this->goToImmediately;
        }

        void setGoToImmediately(bool goToImmediately) {
            this->goToImmediately = goToImmediately;
        }

        int getAutonomousAuthority() {
            return this->autonomousAuthority;
        }

        void setAutonomousAuthority(int autonomousAuthority) {
            this->autonomousAuthority = autonomousAuthority;
        }

        bool getAutonomousPosition() {
            return this->autonomousPosition;
        }

        void setAutonomousPosition(bool autonomousPosition) {
            this->autonomousPosition = autonomousPosition;
        }

		bool getGoToHeight() {
			return this->goToHeight;
		}

		void setGoToHeight(bool goToHeight) {
			this->goToHeight = goToHeight;
		}

		double getLength(double length) {
			return this->length;
		}

		void setLength(double length) {
			this->length = length;
		}

		double getHeight() {
			return height;
		}

		void setHeight(double height) {
			this->height = height;
		}

		double getGearRatio() {
			return gearRatio;
		}

		void setGearRatio(double gearRatio) {
			this->gearRatio = gearRatio;
		}
		
		bool getJammed() {
			return this->jammed;
		}

		void setJammed(bool jammed) {
			this->jammed = jammed;
		}

		bool getDejam() {
			return this->dejam;
		}

		void setDejam(bool dejam) {
			this->dejam = dejam;
		}

		double getDejamTime() {
			return this->dejamTime;
		}

		void setDejamTime(double dejamTime) {
			this->dejamTime = dejamTime;
		}

		double getDejamAuthority() {
			return this->dejamAuthority;
		}

		void setDejamAuthority(double dejamAuthority) {
			this->dejamAuthority = dejamAuthority;
		}

		double getDejamSpeed() {
			return this->dejamSpeed;
		}

		void setDejamSpeed(double dejamSpeed) {
			this->dejamSpeed = dejamSpeed;
		}

		double getDejamStartTime() {
			return this->dejamStartTime;
		}

        ~MotorButton();
    };

} // namespace Pronounce
