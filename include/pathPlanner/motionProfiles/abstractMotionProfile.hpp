#pragma once

#include "utils/pose2d.hpp"

namespace PathPlanner {
	typedef struct MotionProfilePoint_ {
		Pronounce::Pose2D targetPose;
		QSpeed targetSpeed;
		QAcceleration targetAcceleration;
		QCurvature targetCurvature;
	} MotionProfilePoint;

	class AbstractMotionProfile {
	public:
		AbstractMotionProfile() = default;

		virtual MotionProfilePoint update(QTime t) { return {}; }

		virtual QTime getDuration() { return {}; }

		~AbstractMotionProfile() = default;
	};
}