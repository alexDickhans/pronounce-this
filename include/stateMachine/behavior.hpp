#pragma once

#include <string>

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
} // namespace Pronounce
