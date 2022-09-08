#pragma once

#include "spinUp/gameConstants.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/launcher/initLauncher.hpp"

// TODO: Finish implentation

namespace Pronounce {

	struct FlywheelValue {
		Angle turretAngle;
		double flywheelSpeed;
	};

	class FlywheelController {
	private:
		Point targetPosition;
	public:
		FlywheelController(Point targetPosition) {
			this->targetPosition = targetPosition;
		}

		FlywheelValue getFlywheelValue(Point currentPoint, Vector currentVelocity) {
			FlywheelValue flywheelValues;
			Vector targetVector = Vector(&targetPosition, &currentPoint);

			flywheelValues.turretAngle = -(targetVector.getAngle() + 90_deg);
			flywheelValues.flywheelSpeed = flywheelRPM.get(targetVector.getMagnitude().Convert(inch));

			std::cout << "Flywheel Speed: " << flywheelValues.flywheelSpeed << ", Distance: " << targetVector.getMagnitude().Convert(inch) << std::endl;

			return flywheelValues;
		}
		
		~FlywheelController() {}
	};
} // namespace Pronounce
