#pragma once

#include "api.h"
#include "controller.hpp"
#include "button.hpp"

namespace Pronounce {

    class MotorButton : public Button {
    private:
        bool goToImmediately = false;

        bool autonomousButton = false;

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

		double resetPosition(double currentPosition) {
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

        bool getAutonomousButton() {
            return this->autonomousButton;
        }

        void setAutonomousButton(bool autonomousButton) {
            this->autonomousButton = autonomousButton;
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

        ~MotorButton();
    };

} // namespace Pronounce

