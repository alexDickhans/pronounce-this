#pragma once

#include "deadWheel.hpp"
#include "../../../pros/rotation.hpp"

namespace Loco {
	class RotationDeadWheel : public DeadWheel {
	private:
		pros::Rotation& rotation;
	public:
		RotationDeadWheel() = delete;
		RotationDeadWheel(pros::Rotation& rotation, QLength radius) : DeadWheel(radius), rotation(rotation) {
            this->configure();
        }

        void configure() {
            rotation.set_data_rate(5);
            rotation.reset_position();
        }

		void update() final {
			DeadWheel::updateReading(rotation.get_position() * 0.01_deg);
		}

		QLength getTickLength() const override {
			return DeadWheel::radius.getValue() * (0.01_deg).Convert(radian);
		}
	};
}