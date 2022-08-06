#pragma once

#include "units/units.hpp"

// TODO: Add docstrings
// TODO: add comments

namespace Pronounce {
	class Orientation {
	private:
		Angle angle;
	public:
		Orientation(Angle angle) : angle(angle) { }

		virtual void update() {}

		void setAngle(Angle angle) {
			this->angle = angle;
		}

		Angle getAngle() {
			return angle;
		}

		virtual void reset() {
			angle = 0.0;
		}

		~Orientation() {}
	};
} // namespace Pronounce
