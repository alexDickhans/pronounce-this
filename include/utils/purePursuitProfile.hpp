#pragma once

#include "pid/pid.hpp"

namespace Pronounce {
    class PurePursuitProfile
    {
    private:
        PID* pid;
        double lookaheadDistance;
    public:
        PurePursuitProfile();
        PurePursuitProfile(PID* pid);
        PurePursuitProfile(double lookaheadDistance);
        PurePursuitProfile(PID* pid, double lookaheadDistance);

        void operator=(PurePursuitProfile purePursuitProfile) {
            this->pid = purePursuitProfile.getPid();
            this->lookaheadDistance = purePursuitProfile.getLookaheadDistance();
        }

        PID* getPid() {
            return pid;
        }

        void setPid(PID* pid) {
            this->pid = pid;
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
