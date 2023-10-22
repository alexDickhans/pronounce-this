#pragma once

#include <vector>
#include <memory>
#include <string>

#include "pros/serial.hpp"
#include "pros/rtos.hpp"
#include "pros/error.h"

#include "encoding.hpp"
#include "transmitter.hpp"

namespace PT {
	/**
	 * @brief Baudrate of the serial connection between the Brain and the Telemetry radio, only change this if you know what you're doing
	 * 
	 */
	#define TELEMETRY_RADIO_BAUDRATE 115200

	class TelemetryRadio : public Transmitter {
	private:
		std::shared_ptr<pros::Serial> serial;
		Encoding* encoder;

		void transmit(std::vector<uint8_t> data);
		
	public:
		/**
		 * @brief Create a new Telemetry radio object
		 * 
		 * @param port The V5 smart port the telemetry radio is connected to 1-21, 1 indexed
		 */
		TelemetryRadio(std::uint8_t port, Encoding* encoder = new Encoding());

		void transmit(std::string data);

		/**
		 * @brief Get the pros::Serial object used internally for the telemetry smart port
		 * 
		 * @attention UTILIZING THIS SERIAL OBJECT COULD CAUSE DATA LOSS OR DAMAGE TO THE BOARD
		 * 
		 * @return std::shared_ptr<pros::Serial> The internal shared pointer to the pros::Serial object to transmit telemetry data to the board
		 */
		std::shared_ptr<pros::Serial> getSerial();

		~TelemetryRadio();
	};
} // namespace PT
