#pragma once

#include <utility>

#include "stateMachine/behavior.hpp"

namespace Pronounce {

    class Until : public Behavior {
    private:
        BooleanCallback interruptCallback;
	    std::shared_ptr<Behavior> behavior;
    public:
	    Until(const std::shared_ptr<Behavior> &behavior, const BooleanCallback &interruptCallback) : interruptCallback(
			    interruptCallback), behavior(behavior), Behavior(behavior->getName()+"until") {}

	    void initialize() override {
            behavior->initialize();
        }

        void update() override {
            behavior->update();
        }

        void exit() override {
            behavior->exit();
        }

        bool isDone() final {
            return interruptCallback();
        }

        ~Until();
    };

    Until::~Until() = default;
    
} // namespace Pronounce
