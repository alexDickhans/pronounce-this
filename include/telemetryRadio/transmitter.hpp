#pragma once

#include <string>

namespace PT {
	class Transmitter {
	private:
		/* data */
	public:
		Transmitter();

		virtual void transmit(std::string data) { return; }

		~Transmitter();
	};
} // namespace PT
