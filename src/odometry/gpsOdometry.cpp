#include "gpsOdometry.hpp"

namespace Pronounce {
	GpsOdometry::GpsOdometry() : GpsOdometry(nullptr) {
		
	}

	GpsOdometry::GpsOdometry(pros::Gps* gps) {
		this->gps = gps;
	}

	void GpsOdometry::update() {
		if (gps != nullptr && this->goodFix()) {
			pros::c::gps_status_s_t status = gps->get_status();
			this->getPosition()->setX(convertToLocal(status.y));
			this->getPosition()->setY(convertToLocal(status.x));
			this->getPosition()->setTheta(toRadians(status.yaw) + M_PI_2);
		}
	}
	
	GpsOdometry::~GpsOdometry() {
	}
} // namespace Pronounce
