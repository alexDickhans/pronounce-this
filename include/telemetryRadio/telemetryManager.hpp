#pragma once

#include <vector>
#include <memory>
#include <mutex>

#include "pros/rtos.hpp"

#include "encoding.hpp"
#include "logger.hpp"
#include "measurementTypes/measurement.hpp"
#include "measurementTypes/functionMeasurement.hpp"
#include "measurementTypes/variableMeasurement.hpp"
#include "telemetryRadio.hpp"
#include "transmitter.hpp"
#include "usb.hpp"

namespace PT {
	class TelemetryManager {
	private:
		static TelemetryManager* singleton_;
		static pros::Mutex mutex_;

		TelemetryManager();

		std::shared_ptr<pros::Task> updateTask;
		bool taskRunning;

		std::vector<std::shared_ptr<Transmitter>> transmitters;

		std::vector<std::shared_ptr<Measurement>> measurementSources;
		Logger* defaultLogger;

		uint32_t updateTime{10};
	protected:
		void updateScheduler();

		std::string generateTransmission();

		void sendTransmission(std::string transmission);

		void updateMeasurementSources();
	public:
		TelemetryManager(TelemetryManager& other) = delete;
		void operator=(const Logger &) = delete;

		static TelemetryManager* getInstance();

		/**
		 * @brief 
		 * 
		 * If this->taskRunning this method will be called automatically by a 
		 * task created in the constructor and calling it will increase bandwidth and duplicate data.
		 * You can override creating this task by setting the optional parameter "createTask" to 
		 * false in the constructor, or by setting taskRunning to false in the .setTaskRunning(bool) method
		 * 
		 */
		void update();

		/**
		 * @brief 
		*/
		bool enableUpdateScheduler();

		/**
		 * 
		*/
		bool disableUpdateScheduler();

		uint32_t getUpdateTime() {
			return updateTime;
		}

		void setUpdateTime(uint32_t updateTime) {
			this->updateTime = updateTime;
		}

		void addTransmitter(std::shared_ptr<Transmitter> newTransmitter);

		void addMeasurementSource(std::shared_ptr<Measurement> newMeasurementSource);

		~TelemetryManager();
	};	
} // namespace PT
