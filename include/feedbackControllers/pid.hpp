#pragma once

#include <cmath>
#include "utils/utils.hpp"

namespace Pronounce {
	
	/**
	 * @brief PID class
	 * 
	 * @author ad101-lab
	 * 
	 * Hopefully adding FF later
	 * 
	 */
    class PID {
    private:
        double kP;
        double kI;
        double kD;

        double target = 0.0;
        double position;

        double error = 0.0;
        double totalError = 0.0;
        double prevError = 0.0;
        double derivitive;

        double integralBound = 30.0;
        double maxIntegral = 0.3;

        double power;

		bool turnPid = false;
    public:
        PID();
        PID(double kP, double kI, double kD, double target = 0, double position = 0, bool turnPid = false);

        double update();

        void operator=(PID pid) {
            this->kP = pid.getKP();
            this->kI = pid.getKI();
            this->kD = pid.getKD();
            this->integralBound = pid.getIntegralBound();
            this->maxIntegral = pid.getMaxIntegral();
        }

        void operator=(PID* pid) {
            this->kP = pid->getKP();
            this->kI = pid->getKI();
            this->kD = pid->getKD();
            this->integralBound = pid->getIntegralBound();
            this->maxIntegral = pid->getMaxIntegral();
        }

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

		bool getTurnPid() {
			return turnPid;
		}

		void setTurnPid(bool turnPid) {
			this->turnPid = turnPid;
		}

		void reset() {
			this->prevError = 0;
			this->error = 0;
			this->derivitive = 0;
		}

        ~PID();
    };
} // namespace Pronounce



