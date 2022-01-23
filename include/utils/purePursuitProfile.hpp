#pragma once

#include "feedbackControllers/pid.hpp"

namespace Pronounce {
    class PurePursuitProfile {
    private:
        PID* lateralPid;
        PID* turnPid;
        double lookaheadDistance = 0.0;
    public:
        PurePursuitProfile();
        PurePursuitProfile(PID* lateralPid, PID* turnPid);
        PurePursuitProfile(double lookaheadDistance);
        PurePursuitProfile(PID* lateralPid, PID* turnPid, double lookaheadDistance);

        void operator=(PurePursuitProfile purePursuitProfile) {
            this->lateralPid->operator=(purePursuitProfile.getLateralPid());
            this->turnPid->operator=(purePursuitProfile.getTurnPid());
            this->lookaheadDistance = purePursuitProfile.getLookaheadDistance();
        }

        PID* getLateralPid() {
            return lateralPid;
        }

        void setLateralPid(PID* lateralPid) {
            this->lateralPid = lateralPid;
        }

        PID* getTurnPid() {
            return turnPid;
        }

        void setTurnPid(PID* turnPid) {
            this->turnPid = turnPid;
        }

        double getLookaheadDistance() {
            return lookaheadDistance;
        }

        void setLookaheadDistance(double lookaheadDistance) {
            this->lookaheadDistance = lookaheadDistance;
        }

        ~PurePursuitProfile();
    };    
} // namespace Pronounce
