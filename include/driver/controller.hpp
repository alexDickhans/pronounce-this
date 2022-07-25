#pragma once

#include "api.h"
#include "chassis/drivetrain.hpp"
#include "utils/pose2d.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "utils/vector.hpp"

#define STATE_AUTON 0
#define STATE_DRIVER 1

#define PRONOUNCE_CONTROLLER_LEFT 0
#define PRONOUNCE_CONTROLLER_RIGHT 1

namespace Pronounce {

    class Controller: public pros::Controller {
    private:
        uint8_t lastState;

        ContinuousOdometry* odometry;

        // Path to be implemented later.
        
        Drivetrain* drivetrain;

        pros::Task* renderTask;

        bool continueRendering = true;

		uint32_t count;

        // Going to be implemented when the robot is ready
        // pros::Motor* ringMotor;
        // pros::Motor* goalMotor;

        // pros::Vision* vision;

    public:
        Controller(pros::controller_id_e_t id);
        Controller(pros::controller_id_e_t id, ContinuousOdometry* odometry);

        /**
         * Get the degrees that the controller is pointing at
         * 
         * @param joystick The joysick data is coming from
         * 
         * @return The angle in radians 
         */
        double getTheta(int joystick);

        double getMagnitude(int joystick);

        Vector getVector(int joystick);

        void render();
        void renderAuton();
        void renderDisabled();
        void renderDriver();

        void renderFunc();

        void startRenderThread();

        pros::Task* getTask() {
            return this->renderTask;
        }

        void setTask(pros::Task task) {
            this->renderTask = &task;
        }

        bool isRendering() {
            return this->continueRendering;
        }

        void setIsRendering(bool rendering) {
            this->continueRendering = rendering;
        }

        Drivetrain* getDrivetrain() {
            return this->drivetrain;
        }

        void setDrivetrain(Drivetrain* drivetrain) {
            this->drivetrain = drivetrain;
        }

		ContinuousOdometry* getOdometry() {
			return odometry;
		}

		void setOdometry(ContinuousOdometry* odometry) {
			this->odometry = odometry;
		}

		int getLeftX() {
			return this->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		}

		int getRightX() {
			return this->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		}

		int getLeftY() {
			return this->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		}

		int getRightY() {
			return this->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		}

		bool getUp() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_UP);
		}

		bool getDown() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
		}

		bool getLeft() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
		}

		bool getRight() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
		}

		bool getR1() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		}

		bool getR2() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		}

		bool getL1() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		}

		bool getL2() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_L2);
		}

		bool getA() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_A);
		}

		bool getB() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_B);
		}

		bool getX() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_X);
		}

		bool getY() {
			return this->get_digital(pros::E_CONTROLLER_DIGITAL_Y);
		}

        ~Controller();
    };
}
