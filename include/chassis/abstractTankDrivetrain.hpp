#pragma once

#include "abstractDrivetrain.hpp"
#include "utils/utils.hpp"
#include "units/units.hpp"
#include <iostream>

namespace Pronounce {
	class AbstractTankDrivetrain : public AbstractDrivetrain {
	private:
		double trackWidth;
	public:
		AbstractTankDrivetrain() {}
		AbstractTankDrivetrain(double trackWidth) : trackWidth(trackWidth) {}

		virtual QSpeed getSpeed() { return 0.0; }

		double getTrackWidth() {
			return trackWidth;
		}

		void setTrackWidth(double trackWidth) {
			this->trackWidth = trackWidth;
		}

		void driveCurvature(double speed, double curvature) {			
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

		void driveCurvatureVoltage(double speed, double curvature) {			
			double leftSpeed = speed * (2.0 + curvature * trackWidth) / 2.0;
			double rightSpeed = speed * (2.0 - curvature * trackWidth) / 2.0;

			double maxSpeed = max(abs(leftSpeed), abs(rightSpeed));

			if (maxSpeed > abs(speed)) {
				double multiplier = abs(speed) / maxSpeed;
				leftSpeed *= multiplier;
				rightSpeed *= multiplier;
			}

			this->tankSteerVoltage(leftSpeed, rightSpeed);
		}

		virtual void skidSteerVelocity(double speed, double turn) {}

		virtual void tankSteerVelocity(double leftSpeed, double rightSpeed) {}

		virtual void tankSteerVoltage(double leftSpeed, double rightSpeed) {}

		~AbstractTankDrivetrain();
	};
} // namespace Pronounce

