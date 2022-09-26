#pragma once

#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/drivetrain/initDrivetrain.hpp"
#include "chassis/abstractHolonomicDrivetrain.hpp"

namespace Pronounce {
	class OmniMovement : public Behavior {
	private:
		double inputPower = 0;
		Angle angle = 0.0;
		AbstractHolonomicDrivetrain* drivetrain;
	public:
		OmniMovement(std::string name, AbstractHolonomicDrivetrain* drivetrain, double inputPower, Angle angle) : Behavior(name), drivetrain(drivetrain), inputPower(inputPower), angle(angle) {}

		void initialize() {
			this->drivetrain->setDriveVectorVelocity(Vector(inputPower, angle), 0);
		}

		void update() {
			this->drivetrain->setDriveVectorVelocity(Vector(inputPower, angle), 0);
		}

		void exit() {
			this->drivetrain->setDriveVectorVelocity(Vector(), 0);
		}

		bool isDone() {
			return true;
		}

		~OmniMovement() {}
	};
} // namespace Pronounce
