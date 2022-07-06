#pragma once

// Gravity in in/s^2
#define GRAVITY 385.826

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
