#pragma once

#include "api.h"

// TODO: Test code

namespace Pronounce {

	/**
	 * @brief IR beambreak class for ir beambreaks
	 * 
	 * @authors Alex Dickhans (alexDickhans) 
	 */
	class IRBeambreak {
	private:
		/**
		 * @brief The analog sensor
		 * 
		 */
		pros::ADIAnalogIn* irSensor;

		/**
		 * @brief Minimum threshold for a detection
		 * 
		 */
		int32_t threshold;
	public:
		/**
		 * @brief Construct a new IRBeambreak object
		 * 
		 * @param irSensor 
		 */
		IRBeambreak(pros::ADIAnalogIn* irSensor) {
			this->irSensor = irSensor;
		}

		/**
		 * @brief Get the Value from the sensor, thresholded
		 * 
		 * @return true The value is greater than the threshold
		 * @return false The value is less than the threshold
		 */
		bool getValue() {
			return irSensor->get_value() >= threshold;
		}

		/**
		 * @brief Get the a pointer to the sensor
		 * 
		 * @return pros::ADIAnalogIn* Pointer to the sensor
		 */
		pros::ADIAnalogIn* getIrSensor() {
			return irSensor;
		}

		/**
		 * @brief Set the a pointer to the sensor
		 * 
		 * @param irSensor Pointer to the sensor object 
		 */
		void setIrSensor(pros::ADIAnalogIn* irSensor) {
			this->irSensor = irSensor;
		}
		
		~IRBeambreak() {}
	};	
} // namespace Pronounce
