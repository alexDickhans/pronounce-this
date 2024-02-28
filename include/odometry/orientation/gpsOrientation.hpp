#pragma once

#include "orientation.hpp"
#include "pros/gps.hpp"

namespace Pronounce {
	class GpsOrientation : public Orientation {
	private:
		pros::Gps& gps;
		Angle angleOffset;
	public:
		explicit GpsOrientation(pros::Gps &gps, Angle angleOffset) : Orientation(0.0), gps(gps), angleOffset(angleOffset) {}

		void update() override {
			Orientation::setAngle(gps.get_heading_raw() * 1_deg + 90_deg - angleOffset);
		}

		void reset() override {
		}
	};
}