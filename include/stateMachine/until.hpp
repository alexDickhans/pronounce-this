#pragma once

#include <utility>

#include "stateMachine/behavior.hpp"

namespace Pronounce {

    class Until : public Behavior {
    private:
        BooleanCallback interruptCallback;
        Behavior* behavior;
    public:
        Until(std::string name, Behavior* behavior, BooleanCallback  interruptCallback);

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
            return interruptCallback();
        }

		Behavior * rawBehavior() override {
			return behavior;
		}

        ~Until();
    };
    
    Until::Until(std::string name, Behavior* behavior, BooleanCallback  interruptCallback) : Behavior(std::move(name)), interruptCallback(std::move(interruptCallback)) {
		this->behavior = behavior;
    }
    
    Until::~Until() = default;

    Behavior* Behavior::until(const BooleanCallback& booleanCallback) {
        return new Until(this->getName() + "until", this, booleanCallback);
    }
    
} // namespace Pronounce
