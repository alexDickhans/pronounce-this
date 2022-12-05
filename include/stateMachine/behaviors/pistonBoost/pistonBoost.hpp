#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce
{
	enum BoostState {
		boost,
		overfill,
		none
	};

	class PistonBoost : public Behavior {
	private:
		pros::ADIDigitalOut* boostPiston;
		pros::ADIDigitalOut* overfillPiston;

		BoostState boostState;

	public:
		PistonBoost(std::string name, pros::ADIDigitalOut* boostPiston, pros::ADIDigitalOut* overfillPiston, BoostState boostState);

		void initialize() {
			boostPiston->set_value(boostState == BoostState::boost);
			overfillPiston->set_value(boostState == BoostState::overfill);
		}

		void update() {

		}

		void exit() {
			return;
		}

		bool isDone() {
			return false;
		}

		~PistonBoost();
	};
	
	PistonBoost::PistonBoost(std::string name, pros::ADIDigitalOut* boostPiston, pros::ADIDigitalOut* overfillPiston, BoostState boostState) : boostPiston(boostPiston), overfillPiston(overfillPiston), Behavior(name) {
		this->boostState = boostState;
	}
	
	PistonBoost::~PistonBoost()
	{
	}
	
} // namespace Pronounce
