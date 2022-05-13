#pragma once

#include "behavior.hpp"
#include <string>
#include <unordered_map>
#include <mutex>

namespace Pronounce
{
	class StateController : Behavior {
	private:
		Behavior behavior;
		std::unordered_map<std::string, Config> states;
		std::string currentState;
	public:
		StateController();

		void update() {
			
		}

		~StateController();
	};
	
	StateController::StateController(/* args */)
	{
	}
	
	StateController::~StateController()
	{
	}
	
} // namespace Pronounce
