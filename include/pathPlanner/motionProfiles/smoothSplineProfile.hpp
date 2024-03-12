#pragma once

#include "abstractMotionProfile.hpp"

namespace PathPlanner {

	class SmoothSplineProfile : public AbstractMotionProfile {
		std::vector<BezierSegment> bezierSegment;
	protected:
		void calculate() {

		}
	public:
		MotionProfilePoint update(QTime t) override {
			return AbstractMotionProfile::update(t);
		}

		QTime getDuration() override {
			return AbstractMotionProfile::getDuration();
		}
	};

} // Pronounce
