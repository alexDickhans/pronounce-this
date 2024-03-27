#pragma once

#include "winch.hpp"
#include "stateMachine/stateMachine.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {

	auto winchStow = std::make_shared<Winch>(winch, 0.0);
	auto winchUp = std::make_shared<Winch>(winch, 0.7);
	auto winchA = std::make_shared<Winch>(winch, 2.5);
	auto winchC = std::make_shared<Winch>(winch, 3.5);
	auto winchCSequence = std::make_shared<Sequence>("WinchSequence");
	auto winchASequence = std::make_shared<Sequence>("WinchSequence");

	auto winchStateController = std::make_shared<StateController>("Winch", winchStow);
	auto winchStateExtensionController = std::make_shared<StateController>("Winch", winchStow);

	void initWinch() {
		winch.set_zero_position_all(0.0);
		winchCSequence->addState(winchStateController, std::make_shared<Until>(winchUp, [&]() -> bool {return !master->get_digital(E_CONTROLLER_DIGITAL_L2);}));
		winchCSequence->addState(winchStateController, winchC);

		winchASequence->addState(winchStateController, std::make_shared<Until>(winchUp, [&]() -> bool {return !master->get_digital(E_CONTROLLER_DIGITAL_R2);}));
		winchASequence->addState(winchStateController, winchA);
	}
}