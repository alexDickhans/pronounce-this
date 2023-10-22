#pragma once

#include <string>
#include <mutex>
#include <iostream>

#include "pros/rtos.hpp"

#define DEFAULT_LOG_LEVEL LogLevel::warn

namespace PT {

	/**
	 * @brief Log level for the logger, accessed from OkapiLib logging.hpp
	 *
	 */
	enum LogLevel {
		debug = 4,
		info = 3,
		warn = 2,
		error = 1,
		off = 0
	};

	/**
	 * @brief Bare bones singleton logger for use with the telemetry radio
	 *
	 */
	class Logger {
	private:

		Logger();

		static Logger* logger_;
		static pros::Mutex mutex_;

		LogLevel outputLogLevel;

		std::string logBuffer;

	public:
		Logger(Logger& other) = delete;
		void operator=(const Logger &) = delete;

		void debug(std::string debug);

		void info(std::string info);
	
		void warn(std::string warn);

		void error(std::string error);

		/**
		 * @brief Get the Default Logger pointer
		 *
		 * @return Logger* Pointer to the logger singleton object
		 */
		static Logger* getInstance();

		LogLevel getLogLevel();

		void setLogLevel(LogLevel logLevel);

		std::string getNewEntries();

		~Logger();
	};
} // namespace PT
