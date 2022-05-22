#pragma once

#include "behavior.hpp"
#include <vector>

namespace Pronounce
{
	class BehaviorGroup : public Behavior {
	private:
		std::vector<Behavior&> behaviors;
	public:
		BehaviorGroup();

		void initialize() {
			for (int i = 0; i < behaviors.size(); i++) {
				behaviors.at(i).initialize();
			}
		}

		void update() {
			for (int i = 0; i < behaviors.size(); i++) {
				behaviors.at(i).update();
			}
		}

		void exit() {
			for (int i = 0; i < behaviors.size(); i++) {
				behaviors.at(i).exit();
			}
		}

		void addBehavior(Behavior& behavior) {
			behaviors.emplace_back(behavior);
		}

		~BehaviorGroup();
	};	
} // namespace Pronounce
