#pragma once

#ifndef DEBUG
#include "behavior.hpp"
#endif // !DEBUG
#include <vector>

namespace Pronounce {
	/**
	 * @brief A behavior group groups a lot of behaviors together
	 * 
	 * @author Alex Dickhans
	 * 
	 */
	class BehaviorGroup : public Behavior {
	private:
		/**
		 * @brief A list of all the behaviors in this state
		 * 
		 */
		std::vector<Behavior*> behaviors;
	public:
		/**
		 * @brief Construct a new Behavior Group object
		 * 
		 */
		BehaviorGroup() {}

		/**
		 * @brief Initialize all the objects
		 * 
		 */
		void initialize() {
			for (int i = 0; i < behaviors.size(); i++) {
				behaviors.at(i)->initialize();
			}
		}

		/**
		 * @brief Update all the objects
		 * 
		 */
		void update() {
			for (int i = 0; i < behaviors.size(); i++) {
				behaviors.at(i)->update();
			}
		}

		/**
		 * @brief Exit all the objects
		 * 
		 */
		void exit() {
			for (int i = 0; i < behaviors.size(); i++) {
				behaviors.at(i)->exit();
			}
		}

		/**
		 * @brief Add a behavior to the list
		 * 
		 * @param behavior New behavior to add
		 */
		void addBehavior(Behavior* behavior) {
			behaviors.emplace_back(behavior);
		}

		~BehaviorGroup() {}
	};	
} // namespace Pronounce
