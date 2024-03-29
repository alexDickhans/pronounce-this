#pragma once

#include "bezierSegment.hpp"

namespace PathPlanner {
	class SmoothBezierSegment : public BezierSegment {
	private:
		Pronounce::ProfileConstraints profileConstraints;
	public:
		SmoothBezierSegment(const Point &a, const Point &b, const Point &c, const Point &d, bool reversed, bool stopEnd,
		                    const Pronounce::ProfileConstraints &profileConstraints, int granularity = 100) : BezierSegment(a,
		                                                                                                              b,
		                                                                                                              c,
		                                                                                                              d,
		                                                                                                              reversed,
		                                                                                                              stopEnd,
		                                                                                                              granularity),
		                                                                                                profileConstraints(
				                                                                                                profileConstraints) {}

		[[nodiscard]] const Pronounce::ProfileConstraints &getProfileConstraints() const {
			return profileConstraints;
		}

		void setProfileConstraints(const Pronounce::ProfileConstraints &profileConstraints) {
			SmoothBezierSegment::profileConstraints = profileConstraints;
		}

		[[nodiscard]] QLength getDistance() const override {
			return length.getValue();
		}
	};
}