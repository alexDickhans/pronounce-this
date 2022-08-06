#pragma once

#include "utils/vector.hpp"
#include "utils/pose2d.hpp"

// TODO: Add docstrings

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
