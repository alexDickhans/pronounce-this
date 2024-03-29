#pragma once

namespace Pronounce {

	BehaviorGroup stateControllers;

	void initBehaviors() {

		Log("Init");
		stateControllers.addBehavior(drivetrainStateController);

		Log("Init Intake");
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