#pragma once

#include "Eigen/Dense"
#include "locolib/localization/particleFilter/particle.hpp"

namespace Loco {
	class SensorModel {
	private:
		double sensorWeight = 1.0;
	public:
		SensorModel() = default;

		virtual double confidence(Particle const &particle) {
			return 0.0;
		}

		virtual double getWeight() {
			return 1.0;
		}

		void setSensorWeight(double sensorWeight) {
			SensorModel::sensorWeight = sensorWeight;
		}
	};
}