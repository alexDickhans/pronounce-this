#pragma once

namespace Pronounce {

	BehaviorGroup stateControllers;

	void initBehaviors() {
		stateControllers.addBehavior(&drivetrainStateController);
		stateControllers.addBehavior(&intakeExtensionStateController);
		stateControllers.addBehavior(&intakeStateController);
		stateControllers.addBehavior(&leftWingStateController);
		stateControllers.addBehavior(&rightWingStateController);
		stateControllers.addBehavior(&hangReleaseStateController);
		stateControllers.addBehavior(&awpStateController);
	}

	class Enabled : public Behavior {
	private:
	public:
		explicit Enabled() {
		}

		Enabled(std::string name) : Behavior(name) {

		}

		void initialize() override {
			stateControllers.initialize();
		}

		void update() override {
			stateControllers.update();
		}

		void exit() override {
			stateControllers.exit();
		}

		~Enabled() = default;
	};
}