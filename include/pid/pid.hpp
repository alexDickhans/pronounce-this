#pragma once

#include <cmath>
#include "utils/utils.hpp"

namespace Pronounce {
    class PID {
    private:
        double kP;
        double kI;
        double kD;

        double target;
        double position;

        double totalError;
        double prevError;

        double integralBound;
        double maxIntegral;

        double p;
        double i;
        double d;

        double power;
    public:
        PID(double kP, double kI, double kD, double target, double position);

        double update();

        ~PID();
    };
} // namespace Pronounce



