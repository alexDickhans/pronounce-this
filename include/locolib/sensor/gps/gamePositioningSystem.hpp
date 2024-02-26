#pragma once

#include "globalPositioningSystem.hpp"
#include "../../utils/fieldModel.hpp"
#include "pros/gps.hpp"

namespace Loco {
	class GamePositioningSystem : public GlobalPositioningSystem {
		pros::Gps &gps;
	public:
		explicit GamePositioningSystem(pros::Gps &gps) : gps(gps) {

		}

		double confidence(const Particle &particle) final {
			pros::c::gps_status_s_t status = gps.get_status();

			Eigen::Vector2d particlePosition = particle.getState().block<2, 1>(0, 0);
			Eigen::Vector2d measuredPosition = FieldModel::fromGps(status).block<2, 1>(0, 0);

			QLength distance = (particlePosition - measuredPosition).norm();

			return gps.get_error()/sqrt(distance.getValue());
		}

		Eigen::Vector3d getCurrentPosition() {
			pros::c::gps_status_s_t status = gps.get_status();
			return FieldModel::fromGps(status);
		}

		double getWeight() override {
			return GlobalPositioningSystem::getWeight();
		}
	};
}