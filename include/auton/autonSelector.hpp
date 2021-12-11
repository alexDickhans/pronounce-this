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

        Auton preAuton;
        Auton postAuton;

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
            preAuton.run();
            int result = 0;
            if (autonIndex >= autons.size()) {
                result = defaultAuton.run();
            }
            result = autons[autonIndex].run();
            postAuton.run();
            return result;
        }

        Auton getAuton(int index) {
            int result = 0;
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

        Auton getPreAuton() {
            return preAuton;
        }

        void setPreAuton(Auton preAuton) {
            this->preAuton = preAuton;
        }

        Auton getPostAuton() {
            return postAuton;
        }

        void setPostAuton(Auton postAuton) {
            this->postAuton = postAuton;
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


