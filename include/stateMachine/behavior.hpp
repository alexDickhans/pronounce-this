#pragma once

#include <string>
#include "config.hpp"

namespace Pronounce {
	class Behavior {
	private:
	public:
		Behavior();

		virtual void initialize() {}

		virtual void update() {}

		virtual bool isDone() { return false; }

		virtual void exit() {}

		~Behavior();
	};
	
	Behavior::Behavior(/* args */)
	{
	}
	
	Behavior::~Behavior()
	{
	}
	
} // namespace Pronounce
