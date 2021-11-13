#pragma once

#include "api.h"
#include "chassis/drivetrain.hpp"
#include "utils/position.hpp"

#define STATE_AUTON 0
#define STATE_DRIVER 1

#define PRONOUNCE_CONTROLLER_LEFT 0
#define PRONOUNCE_CONTROLLER_RIGHT 1

namespace Pronounce {

    class Controller: public pros::Controller {
    private:
        uint8_t lastState;

        Position* robotPosition;

        // Path to be implemented later.
        
        Drivetrain* drivetrain;

        pros::Task* renderTask;

        bool continueRendering = true;

        // Going to be implemented when the robot is ready
        // pros::Motor* ringMotor;
        // pros::Motor* goalMotor;

        // pros::Vision* vision;

    public:
        Controller(pros::controller_id_e_t id);
        Controller(pros::controller_id_e_t id, Position* robotPosition);

        /**
         * Get the degrees that the controller is pointing at
         * 
         * @param joystick The joysick data is coming from
         * 
         * @return The angle in radians 
         */
        double getTheta(int joystick);

        double getMagnitude(int joystick);

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

        ~Controller();
    };
}
