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
		std::vector<std::shared_ptr<Behavior>> behaviors;
	public:
		/**
		 * @brief Construct a new Behavior Group object
		 * 
		 */
		BehaviorGroup() = default;

		/**
		 * @brief Initialize all the objects
		 * 
		 */
		void initialize() override {
			for (auto & behavior : behaviors) {
				behavior->initialize();
			}
		}

		/**
		 * @brief Update all the objects
		 * 
		 */
		void update() override {
			for (auto & behavior : behaviors) {
				behavior->update();
			}
		}

		/**
		 * @brief Exit all the objects
		 * 
		 */
		void exit() override {
			for (auto & behavior : behaviors) {
				behavior->exit();
			}
		}

		/**
		 * @brief Add a behavior to the list
		 * 
		 * @param behavior New behavior to add
		 */
		void addBehavior(const std::shared_ptr<Behavior>& behavior) {
			behaviors.emplace_back(behavior);
		}

		~BehaviorGroup() = default;
	};	
} // namespace Pronounce
