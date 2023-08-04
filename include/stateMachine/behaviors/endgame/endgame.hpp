#pragma once

#include "api.h"
#include "utils/ADIDigitalOutGroup.hpp"
#include "stateMachine/behavior.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	class Endgame : public Behavior {
	private:
		pros::ADIDigitalOut& digitalOutputGroup;
		bool enabled;
	public:
		Endgame(std::string name, pros::ADIDigitalOut& digitalOutputGroup, bool enabled);

		void initialize() {
			endgameMutex.take();
			digitalOutputGroup.set_value(enabled);
			endgameMutex.give();
		}

		void update() {
			endgameMutex.take();
			digitalOutputGroup.set_value(enabled);
			endgameMutex.give();
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
