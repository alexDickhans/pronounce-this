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
        Behavior* behavior;
    public:
        Continue(std::string name, Behavior* behavior);

        bool isDone() override {
            return false;
        }

        ~Continue() = default;
    };
    
    Continue::Continue(std::string name, Behavior* behavior) : Behavior(std::move(name)), behavior(behavior) {}

    Behavior *Behavior::continueBehavior() {
        return new Continue("continue" + this->getName(), this);
    }
} // namespace Pronounce
