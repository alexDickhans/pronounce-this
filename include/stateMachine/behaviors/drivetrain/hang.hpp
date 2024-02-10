#pragma once

#include "stateMachine/behavior.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "chassis/tankDrive.hpp"

namespace Pronounce {

	std::array<char, 7> hangList = {{'a', 'b', 'c', 'd', 'e', 'f', 'g'}};

	std::unordered_map<char, QLength> hangMap = {
	        {'a', 75_in},
	        {'b', 85_in},
	        {'c', 95_in},
	        {'d', 100_in},
	        {'e', 105_in},
	        {'f', 105_in},
	        {'g', 110_in}};

    class Hang : public Behavior {
    private:
		static QLength hangDistance;
		AbstractTankDrivetrain& drivetrain;
		pros::ADIDigitalOut& ptoPiston;
        double power;
		QLength targetPosition;

		QLength startDistance;

		QTime startTime;

		Hang(AbstractTankDrivetrain& drivetrain, pros::ADIDigitalOut& ptoPiston, double power, QLength targetPosition) : drivetrain(drivetrain), ptoPiston(ptoPiston) {
			this->power = power;
			this->targetPosition = targetPosition;
		}

    public:
		Hang(AbstractTankDrivetrain& drivetrain, pros::ADIDigitalOut& ptoPiston, double power, char tier) : Hang(drivetrain, ptoPiston, power, hangMap[tier]) {

		}

        void initialize() override {
			ptoPiston.set_value(true);
			startDistance = drivetrain.getDistanceSinceReset() + hangDistance;

			startTime = pros::millis() * 1_ms;
        }

        void update() override {
			if (pros::millis() * 1_ms - 100_ms > startTime) {
				drivetrain.tankSteerVoltage(power*1.2e4, power*1.2e4);
			}
        }

		void exit() override {
			drivetrain.tankSteerVoltage(0.0, 0.0);
			hangDistance += drivetrain.getDistanceSinceReset() - startDistance;
		}

		void setTargetPosition(QLength targetPosition) {
			this->targetPosition = targetPosition;
		}

		QLength getTargetPosition() {
			return targetPosition;
		}

		void setTier(char tier) {
			this->setTargetPosition(hangMap[tier]);
		}

		bool hasHung() {
			return hangDistance.getValue() != 0.0;
		}

        bool isDone() override {
            return (drivetrain.getDistanceSinceReset() - startDistance) > targetPosition;
        }
    };

	QLength Hang::hangDistance = 0.0;
}