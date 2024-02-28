#pragma once

#include "units/units.hpp"

namespace Loco {
	class DeadWheel {
	private:
		QLength lastReading;
		QLength delta;
	protected:
		void updateReadingDistance(QLength const newReading) {
			delta = newReading - lastReading;
			lastReading = newReading;
		}

		void updateReading(Angle angle) {
			updateReadingDistance(angle.getValue() * radius.getValue());
		}

		QLength radius;
		QLength tickLength;
	public:
		DeadWheel() = delete;
		explicit DeadWheel(QLength radius) : DeadWheel(radius, 0.0) {

		}

		DeadWheel(QLength radius, QLength tickLength) :
				radius(radius),
				lastReading(0.0),
				delta(0.0) {
			this->tickLength = tickLength;
		}

		virtual void update() {}

		QLength getDisplacement() const {
			return lastReading;
		}

		QLength getDelta() const {
			return delta;
		}

		virtual QLength getTickLength() const {
			return tickLength;
		}
	};
}
