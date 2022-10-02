#pragma once

// Gravity in in/s^2
#define GRAVITY 385.826

// TODO: Finish implmentaiton
// TODO: Evaluate if this is necccesary

namespace Pronounce {
	class Trajectory {
	private:
		double launchAngle;
		double launchSpeed;
		double launchHeight;

		double dragCoefficient;
	public:
		Trajectory(double launchAngle, double launchSpeed, double launchHeight) {
			this->launchAngle = launchAngle;
			this->launchSpeed = launchSpeed;
			this->launchHeight = launchHeight;
		}
		
		double getAltitude(double distance) {
			return launchHeight + 
		}

		~Trajectory() {}
	};
	
} // namespace Pronounce
