#include "continuousOdometry.hpp"

namespace Pronounce {
    ContinuousOdometry::ContinuousOdometry() {
        this->pose = Pose2D();
        this->resetPose = Pose2D();
    }

    ContinuousOdometry::ContinuousOdometry(Pose2D position) {
        this->pose = pose;
        this->resetPose = Pose2D();
    }
    
    ContinuousOdometry::~ContinuousOdometry() {
    }
} // namespace Pronounce

