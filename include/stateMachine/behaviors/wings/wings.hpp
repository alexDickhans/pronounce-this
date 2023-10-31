#pragma once
//
// Created by alex on 9/7/23.
//

#include <utility>

#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class Wings : public Behavior {
	private:
		pros::ADIDigitalOut& leftSolenoid;
		pros::ADIDigitalOut& rightSolenoid;
		bool leftState{false};
		bool rightState{false};
	public:
		Wings(std::string name, pros::ADIDigitalOut& leftSolenoid, pros::ADIDigitalOut& rightSolenoid, bool leftState, bool rightState) : Behavior(std::move(name)), leftSolenoid(leftSolenoid), rightSolenoid(rightSolenoid), leftState(leftState), rightState(rightState) {

		}

		void initialize() override {
			leftSolenoid.set_value(leftState);
			rightSolenoid.set_value(rightState);
		}

		void update() override {
			// none
		}

		void exit() override {

		}

		bool isDone() override {
			return false;
		}
	};
}
