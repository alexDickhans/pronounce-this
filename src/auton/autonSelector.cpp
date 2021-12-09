#include "autonSelector.hpp"

namespace Pronounce {
    AutonSelector::AutonSelector() {
        controller = new Pronounce::Controller(pros::E_CONTROLLER_MASTER);
    }

    AutonSelector::AutonSelector(std::vector<Auton> autons, Auton defaultAuton) : AutonSelector() {
        this->autons = autons;
        this->defaultAuton = defaultAuton;
    }

    AutonSelector::AutonSelector(std::vector<Auton> autons, Auton defaultAuton, Pronounce::Controller* controller) {
        this->autons = autons;
        this->defaultAuton = defaultAuton;
        this->controller = controller;
    }
    
    void AutonSelector::choose() {
        bool done = false;

        controller->setIsRendering(false);

        while (!done) {
            if (controller->get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
                done = true;
            }

            if (controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                autonIndex++;
                if (autonIndex >= autons.size()) {
                    autonIndex = 0;
                }
            }

            if (controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                autonIndex--;
                if (autonIndex < 0) {
                    autonIndex = autons.size() - 1;
                }
            }

            pros::lcd::print(0, "Auton: %s", autons[autonIndex].getName().c_str());

            pros::Task::delay(20);
        }

        controller->setIsRendering(true);
    }

    AutonSelector::~AutonSelector() {
    }
} // namespace Pronounce

