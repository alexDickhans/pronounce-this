#pragma once

namespace Pronounce {

	BehaviorGroup stateControllers;

	void initBehaviors() {

		Log("Init");
		stateControllers.addBehavior(drivetrainStateController);
		stateControllers.addBehavior(frontLeftWingStateController);
		stateControllers.addBehavior(frontRightWingStateController);
		stateControllers.addBehavior(winchStateController);
		stateControllers.addBehavior(winchStateExtensionController);
		stateControllers.addBehavior(catapultStateController);
		stateControllers.addBehavior(intakeExtensionStateController);
		stateControllers.addBehavior(intakeStateController);
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
			Log("Init");
			stateControllers.initialize();
		}

		void update() override {
			Log("Update");
			if (hopperDistanceSensor.get() * 1_mm < 160_mm &&
			    intakeStateController->getName().find(intakeEject->getName()) == -1) {
				Log("Triball detected");
				intakeStateController->sb(intakeHold);
			}

			// See if the distance sensor detects a new object within 1 inch of the sensor
			if (catapultDistance.get() * 1_mm <=
			    1.5_in // see if an object is detected by the distance sensor on the catapult
			    && lastDistance > 1.5_in && pros::millis() * 1_ms - lastCount >
			                                 0.15_s) { // If the last distance sensor reading was greater than an inch indicates that the
				// triball is moving closer to the sensor, meaning that there is a new triball

				// increase the count of shot triballs
				shotTriballs += 1;
				Log("Triball count: " + std::to_string(shotTriballs));

				lastCount = pros::millis() * 1_ms;

				// Set the catapult to try to shoot the triball until it has left the catapult
				catapultStateController->sb(
						std::make_shared<Until>(catapultFire,
						                        [=]() -> bool { return catapultDistance.get() * 1_mm > 1.5_in; }));
			}

			// Store the last distance for the next loop itteration
			lastDistance = catapultDistance.get() * 1_mm;

			stateControllers.update();
		}

		void exit() override {
			Log("Exit");
			stateControllers.exit();
		}

		void resetTriballs() {
			Log("Reset triballs");
			shotTriballs = 0;
		}

		uint32_t getTriballCount() {
			return shotTriballs;
		}

		~Enabled() = default;
	};

	uint32_t Enabled::shotTriballs{0};
}