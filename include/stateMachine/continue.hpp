#pragma once

#include <utility>

#include "stateMachine/behavior.hpp"

namespace Pronounce {
    /**
     * @brief A class that controls a behavior so that it's isDone is never triggered, causing the state to run continuously until it is interrupted by another command
     * 
     */
    class Continue : public Behavior {
    private:
	    std::shared_ptr<Behavior> behavior;
    public:
        Continue(std::string name, std::shared_ptr<Behavior> behavior);

		void initialize() override {
			behavior->initialize();
		}

		void update() override {
			behavior->update();
		}

		void exit() override {
			behavior->exit();
		}

        bool isDone() override {
            return false;
        }

        ~Continue() = default;
    };
    
    Continue::Continue(std::string name, std::shared_ptr<Behavior> behavior) : Behavior(std::move(name)), behavior(behavior) {}
} // namespace Pronounce
