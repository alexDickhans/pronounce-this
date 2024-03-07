#pragma once

#include <string>
#include "pros/rtos.hpp"
#include "pros/rtos.h"
#include "pros/misc.h"

#define Log(msg)   Logger::log(std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__), msg)

namespace Pronounce {

#define INDEX_FILE "/usd/index.bin"

	class Logger {
	private:
		static pros::Task task;

		static uint32_t currentIndex;
		static std::string fileName;
		static std::string buffer;

		static bool installed;

	protected:
		static void update();

		static void newFile();

		static void writeLog(const std::string& write);

	public:
		/**
		 * Singletons should not be cloneable.
		 */
		Logger(Logger &other) = delete;

		/**
		 * Singletons should not be assignable.
		 */
		void operator=(const Logger &) = delete;

		static void log(const std::string& className, const std::string& logData);

		[[nodiscard]] static const std::string &getFileName();

		static uint32_t getCurrentIndex();
	};
} // Pronounce
