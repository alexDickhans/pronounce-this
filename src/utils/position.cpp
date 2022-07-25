#include "pose2d.hpp"

namespace Pronounce {
    Pose2D::Pose2D() : Point() {
        this->angle = 0;
    }

    Pose2D::Pose2D(double angle) : Point() {
        this->angle = angle;
    }

    Pose2D::Pose2D(double x, double y) : Point(x, y) {
        this->angle = 0;
    }

    Pose2D::Pose2D(double x, double y, double angle) : Point(x, y) {
        this->angle = angle;
    }

    Pose2D::~Pose2D() {
    }
} // namespace Pronounce
