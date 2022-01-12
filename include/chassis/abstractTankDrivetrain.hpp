#pragma once

#include "abstractDrivetrain.hpp"
#include "utils/utils.hpp"

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
			double leftSpeed = speed * (2 - curvature * trackWidth) / 2;
			double rightSpeed = speed * (2 + curvature * trackWidth) / 2;

			double maxSpeed = max(leftSpeed, rightSpeed);

			if (maxSpeed > speed) {
				double multiplier = speed / maxSpeed;
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

