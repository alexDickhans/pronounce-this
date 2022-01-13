#pragma once

#include "abstractTankDrivetrain.hpp"
#include "simDrivetrain.hpp"
#include "utils/vector.hpp"
#include "utils/utils.hpp"
#include <algorithm>
#include <iostream>

namespace Pronounce {
	class SimTankDrivetrain : public AbstractTankDrivetrain, public SimDrivetrain {
	private:
		double maxAcceleration;
		double maxSpeed;
		double leftVelocity = 0.0, rightVelocity = 0.0;
		double leftVelocityTarget = 0.0, rightVelocityTarget = 0.0;
		double leftDistance = 0.0, rightDistance = 0.0;
	public:
		SimTankDrivetrain();
		SimTankDrivetrain(double trackWidth);
		SimTankDrivetrain(double trackWidth, double maxAcceleration, double maxSpeed);
		SimTankDrivetrain(double trackWidth, double maxAcceleration, double maxSpeed, Position* position);

		void update();

		void skidSteerVelocity(double speed, double turn) {
			leftVelocityTarget = speed - turn;
			rightVelocityTarget = speed + turn;
		}

		void tankSteerVelocity(double leftSpeed, double rightSpeed) {
			leftVelocityTarget = leftSpeed;
			rightVelocityTarget = rightSpeed;
		}
		
		~SimTankDrivetrain();
	};
} // namespace Pronounce
