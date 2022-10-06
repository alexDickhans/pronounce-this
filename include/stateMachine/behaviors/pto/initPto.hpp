#pragma once

#include "api.h"
#include "intake.hpp"
#include "drive.hpp"
#include "catapult.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/wait.hpp"
#include "stateMachine/sequence.hpp"

// TODO: Clean up
// TODO: move declarations to another place
// TODO: Add comments

namespace Pronounce {

	StateController ptoStateController("PtoStateController", new Behavior());

	void initPto() {
		ptoMutex.take();

		ptoMutex.give();
	}
}