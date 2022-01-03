#pragma once

#include "abstractDrivetrain.hpp"

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

		virtual void skidSteerVelocity(double speed, double turn) {}

		virtual void tankSteerVelocity(double leftSpeed, double rightSpeed) {}

		~AbstractTankDrivetrain();
	};
} // namespace Pronounce

