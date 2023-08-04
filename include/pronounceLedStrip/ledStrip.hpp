#pragma once

#include "api.h"

namespace PronounceLedLib {
	struct AnimationColors {
		uint32_t color1;
		uint32_t color2;
	};

	class LedStripController {
	private:
		AnimationColors colors;
		pros::ADILed ledStrip;
		double duration;
		double lastUpdateTime = 0;
	public:
		LedStripController(pros::ADILed ledStrip, AnimationColors colors, double duration);

		void update();

		void setColors(AnimationColors colors) {
			lastUpdateTime = pros::millis()/1000.0;
			this->colors = colors;
		}

		~LedStripController();
	};	
} // namespace PronounceLedLib

