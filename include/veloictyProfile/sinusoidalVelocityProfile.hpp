#pragma once

#include "velocityProfile.hpp"
#include "units/units.hpp"

namespace Pronounce {
	class SinusoidalVelocityProfile : public VelocityProfile {
	private:
		QAcceleration am;
	public:
		SinusoidalVelocityProfile(/* args */);

		void setMaxAcceleration(QAcceleration am) {
			this->am = am;
		}

		~SinusoidalVelocityProfile();
	};
} // namespace Pronounce
