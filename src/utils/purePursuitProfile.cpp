#include "purePursuitProfile.hpp"

namespace Pronounce {
    PurePursuitProfile::PurePursuitProfile() {
        this->pid = new PID();
    }

    PurePursuitProfile::PurePursuitProfile(PID* pid) {
        this->pid = pid;
    }

    PurePursuitProfile::PurePursuitProfile(double lookaheadDistance) : PurePursuitProfile() {
        this->lookaheadDistance = lookaheadDistance;
    }

    PurePursuitProfile::PurePursuitProfile(PID* pid, double lookaheadDistance) : PurePursuitProfile(pid) {
        this->lookaheadDistance = lookaheadDistance;
    }

    PurePursuitProfile::~PurePursuitProfile() {
    }
} // namespace Pronounce
