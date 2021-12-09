#pragma once

#include "api.h"
#include "auton.hpp"
#include "driver/controller.hpp"
#include <vector>

namespace Pronounce {
    class AutonSelector {
    private:
        std::vector<Auton> autons;
        Auton defaultAuton;

        Pronounce::Controller* controller;

        int autonIndex = 0;
    public:
        AutonSelector();
        AutonSelector(std::vector<Auton> autons, Auton defaultAuton);
        AutonSelector(std::vector<Auton> autons, Auton defaultAuton, Pronounce::Controller* controller);

        void choose();

        int addAuton(Auton auton) {
            autons.push_back(auton);
            return autons.size() - 1;
        }

        int run() {
            if (autonIndex >= autons.size()) {
                return defaultAuton.run();
            }
            return autons[autonIndex].run();
        }

        Auton getAuton(int index) {
            if (index < 0 || index >= autons.size()) {
                return defaultAuton;
            }
            return autons[index];
        }

        std::vector<Auton> getAutons() {
            return autons;
        }

        void setAutons(std::vector<Auton> autons) {
            this->autons = autons;
        }

        Auton getDefaultAuton() {
            return defaultAuton;
        }

        void setDefaultAuton(Auton defaultAuton) {
            this->defaultAuton = defaultAuton;
        }

        void setController(Pronounce::Controller* controller) {
            this->controller = controller;
        }
        
        int getAutonIndex() {
            return autonIndex;
        }

        void setAutonIndex(int autonIndex) {
            this->autonIndex = autonIndex;
        }

        ~AutonSelector();
    };      
}


