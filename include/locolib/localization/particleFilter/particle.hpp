#pragma once

#include <utility>

#include "Eigen/Dense"
#include "pros/gps.hpp"

namespace Loco {
	class Particle {
	private:
		Eigen::Vector3d state;
		double weight{1.0};
	public:
		Particle() = default;
		Particle(Eigen::Vector3d state, double weight = 1.0) {
			this->state = std::move(state);
			this->weight = weight;
		}
        Particle(pros::c::gps_status_s gpsStatus);
		Particle(Eigen::Vector2d state, Angle angle, double weight = 1.0) {
			this->state = Eigen::Vector3d(state.x(), state.y(), angle.getValue());
			this->weight = weight;
		}

		[[nodiscard]] Eigen::Vector3d getState() const {
			return state;
		}

		[[nodiscard]] double getWeight() const {
			return weight;
		}

		void setState(Eigen::Vector3d const state) {
			this->state = state;
		}

		void setAngle(Angle angle) {
			this->state.z() = angle.getValue();
		}

		void setWeight(double const weight) {
			this->weight = weight;
		}

		void clamp(Eigen::Vector2d lowerBound, Eigen::Vector2d upperBound) {
			state = {std::clamp(state.x(), lowerBound.x(), upperBound.x()),
					 std::clamp(state.y(), lowerBound.y(), upperBound.y()),
					 state.z()};
		}
	};
}