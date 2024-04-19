#pragma once

namespace Pronounce {
	class Disabled : public Behavior {
	public:
		Disabled() : Behavior("Disabled") {
		}

		void initialize() override {

		}

		void update() override {

		}

		bool isDone() override {
			return false;
		}

		void exit() override {

		}
	};
}