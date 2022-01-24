#pragma once

#include "api.h"
#include "auton.hpp"
#include "driver/controller.hpp"
#include <vector>

namespace Pronounce {
	class AutonSelector {
	private:
		std::vector<Auton> autons;
		int defaultAuton;

		Auton preAuton;
		Auton postAuton;

		Pronounce::Controller* controller;

		int autonIndex = -1;
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
			int result = this->getAuton(autonIndex).run();
			postAuton.run();
			return result;
		}

		Auton getAuton(int index) {
			int result = 0;
			if (index < 0 || index >= autons.size()) {
				return autons.at(defaultAuton);
			}
			return autons.at(index);
		}

		std::vector<Auton> getAutons() {
			return autons;
		}

		void setAutons(std::vector<Auton> autons) {
			this->autons = autons;
		}

		int getDefaultAuton() {
			return defaultAuton;
		}

		void setDefaultAuton(int defaultAuton) {
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


