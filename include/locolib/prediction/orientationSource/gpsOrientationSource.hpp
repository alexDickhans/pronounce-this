#pragma once

#include "orientationSource.hpp"
#include "pros/gps.hpp"

namespace Loco {
	class GpsOrientationSource : public OrientationSource {
	private:
		pros::Gps& gps;
		Angle angleOffset = 0.0;
	public:
		explicit GpsOrientationSource(pros::Gps& gps, Angle angleOffset) : gps(gps), angleOffset(angleOffset) {
		}

		Angle getAngle() override {
			return (-this->gps.get_heading_raw() * 1_deg) - angleOffset;
		}
	};
}