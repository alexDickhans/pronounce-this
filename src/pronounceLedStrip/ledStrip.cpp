#include "ledStrip.hpp"

namespace PronounceLedLib {
	LedStripController::LedStripController(pros::ADILed ledStrip, AnimationColors colors, double duration) : ledStrip(ledStrip) {
		this->colors = colors;
		this->duration = duration;
		ledStrip.clear_all();
	}

	void LedStripController::update() {
		double timeSinceUpdate = (pros::millis() / 1000.0) - lastUpdateTime;

		timeSinceUpdate = timeSinceUpdate / duration;

		int currentLed = fmod(timeSinceUpdate, 1.0) * ledStrip.length();

		if ((int)timeSinceUpdate % 2 == 1.0) {
			ledStrip[currentLed] = colors.color1;
			ledStrip.update();
		}
		else {
			ledStrip[currentLed] = colors.color2;
			ledStrip.update();
		}
	}

	LedStripController::~LedStripController() {
	}
} // namespace PronounceLedLib
