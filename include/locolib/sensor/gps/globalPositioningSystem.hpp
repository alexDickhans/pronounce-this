#pragma once

#include "../sensorModel.hpp"

namespace Loco {
	class GlobalPositioningSystem : public SensorModel {
	private:

	public:
		double confidence(const Particle &particle) override {
			return SensorModel::confidence(particle);
		}

		double getWeight() override {
			return SensorModel::getWeight();
		}
	};
}
