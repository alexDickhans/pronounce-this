#pragma once

#include "abstractDrivetrain.hpp"
#include "utils/utils.hpp"
#include <iostream>

namespace Pronounce {
	class AbstractTankDrivetrain : public AbstractDrivetrain {
	private:
		double trackWidth;
	public:
		AbstractTankDrivetrain();
		AbstractTankDrivetrain(double trackWidth);

		double getTrackWidth() {
			return trackWidth;
		}

		void setTrackWidth(double trackWidth) {
			this->trackWidth = trackWidth;
		}

		void driveCurvature(double speed, double curvature) {
			std::cout << "Speed: " << speed << " Curvature: " << curvature << std::endl;
			
			double leftSpeed = speed * (2.0 + curvature * trackWidth) / 2.0;
			double rightSpeed = speed * (2.0 - curvature * trackWidth) / 2.0;

			double maxSpeed = max(abs(leftSpeed), abs(rightSpeed));

			if (maxSpeed > abs(speed)) {
				double multiplier = abs(speed) / maxSpeed;
				leftSpeed *= multiplier;
				rightSpeed *= multiplier;
			}

			this->tankSteerVelocity(leftSpeed, rightSpeed);
		}

		virtual void skidSteerVelocity(double speed, double turn) {}

		virtual void tankSteerVelocity(double leftSpeed, double rightSpeed) {}

		~AbstractTankDrivetrain();
	};
} // namespace Pronounce
