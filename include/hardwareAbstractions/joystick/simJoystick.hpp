#pragma once

#include <unistd.h>
#include "joystick.hpp"

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#ifndef __JOYSTICK_HH__
#define __JOYSTICK_HH__

#include <string>
#include <iostream>

#define JS_EVENT_BUTTON 0x01 // button pressed/released
#define JS_EVENT_AXIS   0x02 // joystick moved
#define JS_EVENT_INIT   0x80 // initial state of device

/**
 * Encapsulates all data relevant to a sampled joystick event.
 */
class JoystickEvent
{
public:
  /** Minimum value of axes range */
  static const short MIN_AXES_VALUE = -32768;

  /** Maximum value of axes range */
  static const short MAX_AXES_VALUE = 32767;
  
  /**
   * The timestamp of the event, in milliseconds.
   */
  unsigned int time;
  
  /**
   * The value associated with this joystick event.
   * For buttons this will be either 1 (down) or 0 (up).
   * For axes, this will range between MIN_AXES_VALUE and MAX_AXES_VALUE.
   */
  short value;
  
  /**
   * The event type.
   */
  unsigned char type;
  
  /**
   * The axis/button number.
   */
  unsigned char number;

  /**
   * Returns true if this event is the result of a button press.
   */
  bool isButton()
  {
    return (type & JS_EVENT_BUTTON) != 0;
  }

  /**
   * Returns true if this event is the result of an axis movement.
   */
  bool isAxis()
  {
    return (type & JS_EVENT_AXIS) != 0;
  }

  /**
   * Returns true if this event is part of the initial state obtained when
   * the joystick is first connected to.
   */
  bool isInitialState()
  {
    return (type & JS_EVENT_INIT) != 0;
  }

  /**
   * The ostream inserter needs to be a friend so it can access the
   * internal data structures.
   */
  friend std::ostream& operator<<(std::ostream& os, const JoystickEvent& e);
};

/**
 * Stream insertion function so you can do this:
 *    cout << event << endl;
 */
std::ostream& operator<<(std::ostream& os, const JoystickEvent& e);

/**
 * Represents a joystick device. Allows data to be sampled from it.
 */
class Joystick
{
private:
  void openPath(std::string devicePath, bool blocking=false);
  
  int _fd;
  
public:
  ~Joystick();

  /**
   * Initialises an instance for the first joystick: /dev/input/js0
   */
  Joystick();

  /**
   * Initialises an instance for the joystick with the specified,
   * zero-indexed number.
   */
  Joystick(int joystickNumber);

  /**
   * Initialises an instance for the joystick device specified.
   */
  Joystick(std::string devicePath);

  /**
   * Joystick objects cannot be copied
   */
  Joystick(Joystick const&) = delete;

  /**
   * Joystick objects can be moved
   */
  Joystick(Joystick &&) = default;

  /**
   * Initialises an instance for the joystick device specified and provide
   * the option of blocking I/O.
   */
  Joystick(std::string devicePath, bool blocking);
 
  /**
   * Returns true if the joystick was found and may be used, otherwise false.
   */
  bool isFound();
  
  /**
   * Attempts to populate the provided JoystickEvent instance with data
   * from the joystick. Returns true if data is available, otherwise false.
   */
  bool sample(JoystickEvent* event);
};

#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include "unistd.h"

Joystick::Joystick()
{
  openPath("/dev/input/js0");
}

Joystick::Joystick(int joystickNumber)
{
  std::stringstream sstm;
  sstm << "/dev/input/js" << joystickNumber;
  openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath)
{
  openPath(devicePath);
}

Joystick::Joystick(std::string devicePath, bool blocking)
{
  openPath(devicePath, blocking);
}

void Joystick::openPath(std::string devicePath, bool blocking)
{
  // Open the device using either blocking or non-blocking
  _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
}

bool Joystick::sample(JoystickEvent* event)
{
  int bytes = read(_fd, event, sizeof(*event)); 

  if (bytes == -1)
    return false;

  // NOTE if this condition is not met, we're probably out of sync and this
  // Joystick instance is likely unusable
  return bytes == sizeof(*event);
}

bool Joystick::isFound()
{
  return _fd >= 0;
}

Joystick::~Joystick()
{
  close(_fd);
}

std::ostream& operator<<(std::ostream& os, const JoystickEvent& e)
{
  os << "type=" << static_cast<int>(e.type)
     << " number=" << static_cast<int>(e.number)
     << " value=" << static_cast<int>(e.value);
  return os;
}

namespace Pronounce
{

class SimJoystick {
private:
	Joystick joystick;
	std::int32_t leftX;
	std::int32_t leftY;
	std::int32_t rightX;
	std::int32_t rightY;

	bool leftBumper = false;
	bool rightBumper = false;
	bool leftTrigger = false;
	bool rightTrigger = false;
	bool up = false;
	bool down = false;
	bool left = false;
	bool right = false;
	bool x = false;
	bool y = false;
	bool a = false;
	bool b = false;

	bool leftBumperNewPress = false;
	bool rightBumperNewPress = false;
	bool leftTriggerNewPress = false;
	bool rightTriggerNewPress = false;
	bool upNewPress = false;
	bool downNewPress = false;
	bool leftNewPress = false;
	bool rightNewPress = false;
	bool xNewPress = false;
	bool yNewPress = false;
	bool aNewPress = false;
	bool bNewPress = false;

public:
	SimJoystick(int joystick);

	std::int32_t is_connected() {
		return joystick.isFound();
	}

	std::int32_t get_analog(controller_analog_e_t channel) {
		switch (channel) {
		case E_CONTROLLER_ANALOG_LEFT_X:
			return leftX;
		case E_CONTROLLER_ANALOG_LEFT_Y:
			return leftY;
		case E_CONTROLLER_ANALOG_RIGHT_X:
			return rightX;
		case E_CONTROLLER_ANALOG_RIGHT_Y:
			return rightY;

		default:
			return 0;
			break;
		}
	}

	std::int32_t get_digital(controller_digital_e_t channel) {
		switch (channel)
		{
		case E_CONTROLLER_DIGITAL_B:
			return b;
		case E_CONTROLLER_DIGITAL_A:
			return a;
		case E_CONTROLLER_DIGITAL_Y:
			return y;
		case E_CONTROLLER_DIGITAL_X:
			return x;
		case E_CONTROLLER_DIGITAL_L1:
			return leftBumper;
		case E_CONTROLLER_DIGITAL_R1:
			return rightBumper;
		case E_CONTROLLER_DIGITAL_L2:
			return leftTrigger;
		case E_CONTROLLER_DIGITAL_R2:
			return rightTrigger;
		case E_CONTROLLER_DIGITAL_UP:
			return up;
		case E_CONTROLLER_DIGITAL_DOWN:
			return down;
		case E_CONTROLLER_DIGITAL_LEFT:
			return left;
		case E_CONTROLLER_DIGITAL_RIGHT:
			return right;
		default:
			return false;
		}
	}

	std::int32_t get_digital_new_press(controller_digital_e_t channel) {
		switch (channel)
		{
		case E_CONTROLLER_DIGITAL_B:
			if (bNewPress) {
				bNewPress = false;
				return !bNewPress;
			}
		case E_CONTROLLER_DIGITAL_A:
			if (aNewPress) {
				aNewPress = false;
				return !aNewPress;
			}
		case E_CONTROLLER_DIGITAL_Y:
			if (yNewPress) {
				yNewPress = false;
				return !yNewPress;
			}
		case E_CONTROLLER_DIGITAL_X:
			if (xNewPress) {
				xNewPress = false;
				return !xNewPress;
			}
		case E_CONTROLLER_DIGITAL_L1:
			if (leftBumperNewPress) {
				leftBumperNewPress = false;
				return !leftBumperNewPress;
			}
		case E_CONTROLLER_DIGITAL_R1:
			if (rightBumperNewPress) {
				rightBumperNewPress = false;
				return !rightBumperNewPress;
			}
		case E_CONTROLLER_DIGITAL_L2:
			if (leftTriggerNewPress) {
				leftTriggerNewPress = false;
				return !leftTriggerNewPress;
			}
		case E_CONTROLLER_DIGITAL_R2:
			if (rightTriggerNewPress) {
				rightTriggerNewPress = false;
				return !rightTriggerNewPress;
			}
		case E_CONTROLLER_DIGITAL_UP:
			if (upNewPress) {
				upNewPress = false;
				return !upNewPress;
			}
		case E_CONTROLLER_DIGITAL_DOWN:
			if (downNewPress) {
				downNewPress = false;
				return !downNewPress;
			}
		case E_CONTROLLER_DIGITAL_LEFT:
			if (leftNewPress) {
				leftNewPress = false;
				return !leftNewPress;
			}
		case E_CONTROLLER_DIGITAL_RIGHT:
			if (rightNewPress) {
				rightNewPress = false;
				return !rightNewPress;
			}
		default:
			return false;
		}
		return false;
	}

	void update() {
		JoystickEvent event;
		if (joystick.sample(&event))
		{
			if (event.isButton())
			{
				switch (event.number)
				{
				case 0:
					b = event.value != 0;
					bNewPress = event.value != 0;
					break;
				case 1:
					a = event.value != 0;
					aNewPress = event.value != 0;
					break;
				case 2:
					y = event.value != 0;
					yNewPress = event.value != 0;
					break;
				case 3:
					x = event.value != 0;
					xNewPress = event.value != 0;
					break;
				case 4:
					leftBumper = event.value != 0;
					leftBumperNewPress = event.value != 0;
					break;
				case 5:
					rightBumper = event.value != 0;
					rightBumperNewPress = event.value != 0;
					break;
				case 6:
					leftTrigger = event.value != 0;
					leftTriggerNewPress = event.value != 0;
					break;
				case 7:
					rightTrigger = event.value != 0;
					rightTriggerNewPress = event.value != 0;
					break;
				case 12:
					up = event.value != 0;
					upNewPress = event.value != 0;
					break;
				case 13:
					down = event.value != 0;
					downNewPress = event.value != 0;
					break;
				case 14:
					left = event.value != 0;
					leftNewPress = event.value != 0;
					break;
				case 15:
					right = event.value != 0;
					rightNewPress = event.value != 0;
					break;
				default:
					break;
				}
				// printf("Button %u is %s\n",
				// 	event.number,
				// 	event.value == 0 ? "up" : "down");
			}
			else if (event.isAxis())
			{
				switch (event.number)
				{
				case 0:
					leftX = event.value / 258;
					break;
				case 1:
					leftY = event.value / 258;
					break;
				case 2:
					rightX = event.value / 258;
					break;
				case 3:
					rightY = event.value / 258;
					break;

				default:
					break;
				}
			}
		}
	}

	~SimJoystick();
};

} // namespace Pronounce
