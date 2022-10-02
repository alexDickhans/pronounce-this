#pragma once

#include "api.h"
#include "utils/ADIDigitalOutGroup.hpp"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class Endgame : public Behavior {
	private:
		pros::ADIDigitalOut& digitalOutputGroup;
		bool enabled;
	public:
		Endgame(std::string name, pros::ADIDigitalOut& digitalOutputGroup, bool enabled);

		void initialize() {
			digitalOutputGroup.set_value(enabled);
		}

		void update() {
			digitalOutputGroup.set_value(enabled);
		}

		void exit() {

		}

		bool isDone() {
			return false;
		}

		~Endgame();
	};
	
	Endgame::Endgame(std::string name, pros::ADIDigitalOut& digitalOutputGroup, bool enabled) : digitalOutputGroup(digitalOutputGroup), enabled(enabled), Behavior(name) {
	}
	
	Endgame::~Endgame() {
	}
} // namespace Pronounce
