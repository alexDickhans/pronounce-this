#pragma once

#include <string>


namespace Pronounce {

    typedef int (*AutonFunction)();

    int nullAutonFunc();

    class Auton {
    private:
        std::string name;
        AutonFunction func;
    public:
        Auton();
        Auton(std::string name, AutonFunction func);

        int run();

        std::string getName() {
            return name;
        }

        void setName(std::string name) {
            this->name = name;
        }

        AutonFunction getFunc() {
            return func;
        }

        void setFunc(AutonFunction func) {
            this->func = func;
        }

        ~Auton();
    };
} // namespace Pronounce


