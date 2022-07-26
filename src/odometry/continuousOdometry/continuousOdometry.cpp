#include "continuousOdometry.hpp"

namespace Pronounce {
    ContinuousOdometry::ContinuousOdometry() {
        this->pose = new Pose2D();
        this->resetPose = new Pose2D();
    }

    ContinuousOdometry::ContinuousOdometry(Pose2D* position) {
        this->pose = pose;
        this->resetPose = new Pose2D();
    }
    
    ContinuousOdometry::~ContinuousOdometry() {
    }
} // namespace Pronounce

