// Enable debug for some classes
#define DEBUG

#include "../include/stateMachine/behavior.hpp"
#include "../include/stateMachine/wait.hpp"
#include "../include/stateMachine/stateController.hpp"
#include "../include/stateMachine/sequence.hpp"
#include "../include/stateMachine/behaviorGroup.hpp"
#include "testBehvaior.hpp"
#include <iostream>
#include <chrono>
#include <thread>

// Import timers so it works on both linux and windows
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using namespace Pronounce;

/**
 * @brief Tester class for the state machine
 * 
 * @return int 
 */
int main() {
	// Create state objects to test the state machine with
	TestBehavior codeOrange = TestBehavior("Code Orange");
	TestBehavior codeGreen = TestBehavior("Code Green");
	TestBehavior defaultState = TestBehavior("Default");

	// Create wait states to test the wait class with
	Wait codeOrange2Seconds = Wait(&codeOrange, 2000);
	Wait codeGreen3Seconds = Wait(&codeGreen, 3000);

	// Create a state machine for both the state extensions(Parallel/sequences) and a state machine for example a subsystem
	StateController stateController = StateController(&defaultState);
	StateController stateExtensions = StateController(new Behavior());

	// Create a test sequence
	Sequence codeBrown = Sequence();

	// Add states to the sequence
	codeBrown.addState(&stateController, &codeOrange2Seconds);
	codeBrown.addState(&stateController, &codeGreen3Seconds);

	// Group all the behaviors to make it easier to use
	BehaviorGroup behaviorGroup = BehaviorGroup();

	// Add the behaviors
	//! Have to be in this order or else there will be an error
	behaviorGroup.addBehavior(&stateExtensions);
	behaviorGroup.addBehavior(&stateController);

	// Initialize all the behaviors
	behaviorGroup.initialize();

	for (int i = 0; i < 150; i++) {
		// Update all the behaviors
		behaviorGroup.update();

		// Print out the index
		std::cout << i << ": ";

		// Start codeOrange behavior at index 10
		if (i == 10) {
			stateController.setCurrentBehavior(&codeOrange);
		}

		// End the behavior at index 20 by telling the state it is done
		if (i == 20) {
			codeOrange.setDone(true);
		}

		// Test if it will stop quickly
		if (i == 30) {
			stateController.setCurrentBehavior(&codeOrange);
		}

		// Start the behavior again
		if (i == 40) {
			codeOrange.setDone(false);
			stateController.setCurrentBehavior(&codeOrange);
		}

		// Start the default behavior by giving a pointer to it
		if (i == 50) {
			stateController.setCurrentBehavior(&defaultState);
		}

		// Run the wait state for 2 seconds or 20 frames
		if (i == 60) {
			stateController.setCurrentBehavior(&codeOrange2Seconds);
		}

		// Start the sequence.
		if (i == 90) {
			stateExtensions.setCurrentBehavior(&codeBrown);
		}

		// Delay for 100 seconds so that wait sequences will work
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
	// Exit all the states
	behaviorGroup.exit();
}