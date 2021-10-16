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

        double error;
        double totalError;
        double prevError;
        double derivitive;

        double integralBound = 30.0;
        double maxIntegral = 0.3;

        double p;
        double i;
        double d;

        double power;
    public:
        PID();
        PID(double kP, double kI = 0, double kD = 0, double target = 0, double position = 0);

        double getDerivitive() {
            return derivitive;
        }

        double getError() {
            return error;
        }

        double getPower() {
            return power;
        }

        void setPower(double power) {
            this->power = power;
        }

        double getPosition() {
            return this->position;
        }

        void setPosition(double position) {
            this->position = position;
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

        double getIntegralBound() {
            return this->integralBound;
        }

        void setIntegralBound(double integralBound) {
            this->integralBound = integralBound;
        }

        double getMaxIntegral() {
            return this->maxIntegral;
        }

        void setMaxIntegral(double maxIntegral) {
            this->maxIntegral = maxIntegral;
        }

        double update();

        ~PID();
    };
} // namespace Pronounce



