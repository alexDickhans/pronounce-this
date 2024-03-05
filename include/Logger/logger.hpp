#pragma once

#include <string>
#include "pros/rtos.hpp"

namespace Pronounce {

#define INDEX_FILE "/usd/index.bin"

	class Logger {
	private:
		static Logger * pinstance_;
		static pros::Mutex mutex_;
		std::shared_ptr<pros::Task> task_ = nullptr;

		uint32_t currentIndex;
		std::string fileName;
		std::string buffer;

		FILE* logFile;

	protected:
		Logger();
		~Logger() = default;

		void update();

		void writeLog(const std::string& write) {
			buffer.append(write);
		}

	public:
		/**
		 * Singletons should not be cloneable.
		 */
		Logger(Logger &other) = delete;

		/**
		 * Singletons should not be assignable.
		 */
		void operator=(const Logger &) = delete;

		static Logger* getInstance();

		void log(const std::string& className, const std::string& logData);

		[[nodiscard]] const std::string &getFileName() const;

		uint32_t getCurrentIndex() const;
	};
} // Pronounce
