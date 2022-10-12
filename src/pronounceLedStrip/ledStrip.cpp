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

		std::cout << currentLed << std::endl;

		if ((int)timeSinceUpdate % 2 == 1.0) {
			std::cout << "Color 1" << std::endl;
			ledStrip[currentLed] = colors.color1;
			ledStrip.update();
		}
		else {
			std::cout << "Color 2" << std::endl;
			ledStrip[currentLed] = colors.color2;
			ledStrip.update();
		}
	}

	LedStripController::~LedStripController() {
	}
} // namespace PronounceLedLib
