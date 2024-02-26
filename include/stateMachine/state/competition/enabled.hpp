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