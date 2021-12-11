#include "auton.hpp"

namespace Pronounce {
    int nullAutonFunc() {
        return 0;
    }

    Auton::Auton(/* args */) {
        this->name = "";
        this->func = nullAutonFunc;
    }

    Auton::Auton(std::string name, AutonFunction func) {
        this->name = name;
        this->func = func;
    }

    int Auton::run() {
        return func();
    }

    Auton::~Auton() {
    }
} // namespace Pronounce
