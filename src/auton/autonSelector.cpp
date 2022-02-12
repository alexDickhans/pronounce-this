#include "autonSelector.hpp"

namespace Pronounce {
	AutonSelector::AutonSelector() {
		this->controller = new Pronounce::Controller(pros::E_CONTROLLER_MASTER);
		this->preAuton = Auton();
		this->postAuton = Auton();
		this->autons = std::vector<Auton>();
		this->defaultAuton = 0;
	}

	AutonSelector::AutonSelector(std::vector<Auton> autons, int defaultAuton) {
		this->controller = new Pronounce::Controller(pros::E_CONTROLLER_MASTER);
		this->preAuton = Auton();
		this->postAuton = Auton();
		this->autons = autons;
		this->defaultAuton = defaultAuton;
	}

	AutonSelector::AutonSelector(std::vector<Auton> autons, int defaultAuton, Pronounce::Controller* controller) {
		this->autons = autons;
		this->defaultAuton = defaultAuton;
		this->controller = controller;
	}

	void AutonSelector::choose() {
		bool done = false;

		controller->setIsRendering(false);

		printf("Auton selector\n");

		uint32_t count = 0;

		while (!done) {
			controller->clear();

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

			if (count % 3 == 0) {
				controller->set_text(0, 0, autons[autonIndex].getName());
			}

			count++;

			pros::Task::delay(50);

		}

		printf("Auton selected: %s", autons[autonIndex].getName().c_str());

		controller->setIsRendering(true);
	}

	AutonSelector::~AutonSelector() {
	}
} // namespace Pronounce

