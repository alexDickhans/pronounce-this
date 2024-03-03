#pragma once

namespace Pronounce {

	BehaviorGroup stateControllers;

	void initBehaviors() {
		stateControllers.addBehavior(&drivetrainStateController);
		stateControllers.addBehavior(&intakeExtensionStateController);
		stateControllers.addBehavior(&leftWingStateController);
		stateControllers.addBehavior(&rightWingStateController);
		stateControllers.addBehavior(&hangStateController);
		stateControllers.addBehavior(&awpStateController);

		if (isSkills) {
			stateControllers.addBehavior(&catapultStateController);
		} else {
			stateControllers.addBehavior(&intakeStateController);
		}
	}

	class Enabled : public Behavior {
	private:
		static uint32_t shotTriballs;
		QLength lastDistance{0.0};
		QTime lastCount = 0.0;
	public:
		explicit Enabled() {
		}

		Enabled(std::string name) : Behavior(name) {

		}

		void initialize() override {
			stateControllers.initialize();
		}

		void update() override {
			if (hopperDistanceSensor.get() * 1_mm < 8_in &&
			    intakeStateController.getName().find(intakeEject.getName()) == -1) {
				intakeStateController.setCurrentBehavior(&intakeHold);
			}

			// See if the distance sensor detects a new object within 1 inch of the sensor
			if (catapultDistance.get() * 1_mm <
			    0.75_in // see if an object is detected by the distance sensor on the catapult
			    && lastDistance > 0.75_in && pros::millis()*1_ms - lastCount > 0.25_s) { // If the last distance sensor reading was greater than an inch indicates that the
				// triball is moving closer to the sensor, meaning that there is a new triball

				// increase the count of shot triballs
				shotTriballs += 1;

				lastCount = pros::millis() * 1_ms;

				// Set the catapult to try to shoot the triball until it has left the catapult
				catapultStateController.setCurrentBehavior(
						catapultFire.until([=]() -> bool { return catapultDistance.get() * 1_mm > 0.75_in; }));
			}

			// Store the last distance for the next loop itteration
			lastDistance = catapultDistance.get() * 1_mm;

			stateControllers.update();
		}

		void exit() override {
			stateControllers.exit();
		}

		void resetTriballs() {
			shotTriballs = 0;
		}

		uint32_t getTriballCount() {
			return shotTriballs;
		}

		~Enabled() = default;
	};

	uint32_t Enabled::shotTriballs{0};
}