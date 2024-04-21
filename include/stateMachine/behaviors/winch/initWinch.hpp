#pragma once

#include "winch.hpp"
#include "stateMachine/stateMachine.hpp"
#include "hardware/hardware.hpp"
#include "winchVoltage.hpp"

namespace Pronounce {

	auto winchStow = std::make_shared<WinchVoltage>(winch, 0);
	auto winchUp = std::make_shared<Winch>(winch, 0.8);
	auto winchUpWait = std::make_shared<Wait>(winchUp, 1.0_s);
	auto winchA = std::make_shared<Winch>(winch, 2.5);
	auto winchC = std::make_shared<Winch>(winch, 3.5);
	auto winchCSequence = std::make_shared<Sequence>("WinchSequence");
	auto winchASequence = std::make_shared<Sequence>("WinchSequence");

	auto winchStateController = std::make_shared<StateController>("Winch", winchStow);
	auto winchStateExtensionController = std::make_shared<StateController>("WinchExtension", std::make_shared<Behavior>());

	void initWinch() {
		winch.set_zero_position_all(0.0);
		winchCSequence->addState(winchStateController, winchUpWait);
		winchCSequence->addState(winchStateController, std::make_shared<Until>(winchUp, [&]() -> bool {return !master.get_digital(E_CONTROLLER_DIGITAL_R2);}));
		winchCSequence->addState(winchStateController, std::make_shared<Wait>(winchC, 5_s));

		winchASequence->addState(winchStateController, winchUpWait);
		winchASequence->addState(winchStateController, std::make_shared<Until>(winchUp, [&]() -> bool {return !master.get_digital(E_CONTROLLER_DIGITAL_L2);}));
		winchASequence->addState(winchStateController, std::make_shared<Wait>(winchA, 5_s));
	}
}