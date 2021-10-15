#pragma once

#include <cmath>
#include "utils/utils.hpp"

namespace Pronounce {


    double signum_c(double x);

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

        double getPower() {
            return power;
        }

        void setPower(double power) {
            this->power = power;
        }

        double getTarget() {
            return target;
        }

        void setTarget(double target) {
            this->target = target;
        }
        
        double getKP() {
            return kP;
        }

        void setKP(double kP) {
            this->kP = kP;
        }

        double getKI() {
            return kI;
        }

        void setKI(double kI) {
            this->kI = kI;
        }

        double getKD() {
            return kD;
        }

        void setKD(double kD) {
            this->kD = kD;
        }

        double update();

        ~PID();
    };
} // namespace Pronounce



