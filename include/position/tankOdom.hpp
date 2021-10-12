#pragma once

#include "utils/position.hpp"
#include "odomWheel.hpp"
#include "utils/utils.hpp"
#include "api.h"

using namespace Pronounce;

namespace Pronounce {
    class TankOdom {
    private:
        Position* position;

        OdomWheel* leftPivot;
        OdomWheel* rightPivot;

        pros::Imu* imu;

    public:
        TankOdom(OdomWheel* leftPivot, OdomWheel* rightPivot, pros::Imu* imu);

        void update();

        Position getPosition() {
            return *this->position;
        }

        void setPostition(Position position) {
            this->position = &position;
        }

        ~TankOdom();
    };
} // namespace Pronounce
