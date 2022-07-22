#pragma once

#include "utils/vector.hpp"
#include "utils/pose2d.hpp"

namespace Pronounce {

	class InterruptOdometry {
	private:
	public:
		InterruptOdometry() {}

		virtual bool positionReady(Pose2D currentPose, Vector velocity) { return false; }

		virtual Pose2D getPosition(Pose2D currentPose, Vector velocity) {
			return currentPose;
		}

		~InterruptOdometry() {}
	};
} // namespace Pronounce
