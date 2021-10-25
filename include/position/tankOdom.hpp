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

        OdomWheel* odomWheel;

        pros::Imu* imu;

        double tuningFactor = 1.0;

    public:
        TankOdom(OdomWheel* odomWheel, pros::Imu* imu);

        double getTuringFactor() {
            return this->tuningFactor;
        }

        void setTuningFactor(double tuningFactor) {
            this->tuningFactor = tuningFactor;
        }

        OdomWheel* getOdomWheel() {
            return this->odomWheel;
        }

        void setOdomWheel(OdomWheel* odomWheel) {
            this->odomWheel = odomWheel;
        }

        void update();

        void reset();

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
