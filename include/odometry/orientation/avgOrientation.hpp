#pragma once

#include "units/units.hpp"
#include "orientation.hpp"
#include <vector>

namespace Pronounce {
	class AvgOrientation : public Orientation {
	private:
		std::vector<Orientation&> orientations;
	public:
		AvgOrientation();

		void update() {
			Angle total = 0.0;

			for (int i = 0; i < orientations.size(); i ++) {
				orientations.at(i).update();
				total += orientations.at(i).getAngle();
			}

			this->setAngle(total/orientations.size());
		}

		void reset() {
			for (int i = 0; i < orientations.size(); i++) {
				orientations.at(i).reset();
			}
		}

		~AvgOrientation();
	};
} // namespace Pronounce
