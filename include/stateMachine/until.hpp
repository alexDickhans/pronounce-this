#pragma once

#include "stateMachine/behavior.hpp"

namespace Pronounce {

    class Until : public Behavior {
    private:
        BooleanCallback interruptCallback;
        Behavior* behavior;
    public:
        Until(std::string name, BooleanCallback interruptCallback);

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

        ~Until();
    };
    
    Until::Until(std::string name, BooleanCallback interruptCallback) {
    }
    
    Until::~Until() {
    }

    Behavior* Behavior::until(BooleanCallback booleanCallback) {
        return new Until(this->getName() + "until", booleanCallback);
    }
    
} // namespace Pronounce
