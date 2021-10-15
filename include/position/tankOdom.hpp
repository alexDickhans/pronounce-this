#pragma once

#include "utils/position.hpp"
#include "odomWheel.hpp"
#include "motorOdom.hpp"
#include "utils/utils.hpp"
#include "api.h"
#include <string>

using namespace Pronounce;

namespace Pronounce {
    class TankOdom {
    private:
        Position* position;

        MotorOdom* leftPivot;
        MotorOdom* rightPivot;

        pros::Imu* imu;

    public:
        TankOdom(MotorOdom* leftPivot, MotorOdom* rightPivot, pros::Imu* imu);

        void update();

        Position* getPosition() {
            return this->position;
        }

        void setPosition(Position* position) {
            this->position = position;
        }

        std::string to_string(){
            return this->getPosition()->to_string();
        }

        ~TankOdom();
    };
} // namespace Pronounce
