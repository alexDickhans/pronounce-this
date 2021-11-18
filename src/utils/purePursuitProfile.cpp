#include "purePursuitProfile.hpp"

namespace Pronounce {
    PurePursuitProfile::PurePursuitProfile() {
        this->lateralPid = new PID();
        this->turnPid = new PID();
    }

    PurePursuitProfile::PurePursuitProfile(PID* lateralPid, PID* turnPid) {
        this->lateralPid = lateralPid;
        this->turnPid = turnPid;
    }

    PurePursuitProfile::PurePursuitProfile(double lookaheadDistance) : PurePursuitProfile() {
        this->lookaheadDistance = lookaheadDistance;
    }

    PurePursuitProfile::PurePursuitProfile(PID* lateralPid, PID* turnPid, double lookaheadDistance) : PurePursuitProfile(lateralPid, turnPid) {
        this->lookaheadDistance = lookaheadDistance;
    }

    PurePursuitProfile::~PurePursuitProfile() {
    }
} // namespace Pronounce
