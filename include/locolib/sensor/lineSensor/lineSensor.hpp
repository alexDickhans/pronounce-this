#pragma once

#include <utility>

#include "../sensorModel.hpp"
#include "Eigen/Eigen"

namespace Loco {
	class LineSensor : public SensorModel {
	private:
		FieldModel& fieldModel;
		Eigen::Vector2d sensorPosition;
	public:
		explicit LineSensor(FieldModel& fieldModel, Eigen::Vector2d sensorPosition) : fieldModel(fieldModel), sensorPosition(std::move(sensorPosition)) {

		}

		virtual bool getReading() {
			return false;
		}

		double confidence(const Loco::Particle &particle) final {
			bool observedReading = this->getReading();

			Eigen::Vector3d particleLocation = particle.getState();
			Eigen::Vector2d sensorLocation = Eigen::Matrix2d({{cos(particleLocation.z()), sin(particleLocation.z())}, {sin(particleLocation.z()), cos(particleLocation.z())}}) * sensorPosition;
			sensorLocation += particleLocation.head<2>();
			bool predictedReading = fieldModel.isOverLine(sensorLocation);

			return observedReading && predictedReading ? 1.0 : 0.02;
		}
	};
}