#pragma once

#include <string>

#include "measurement.hpp"

namespace PT {
    template <typename T>
    class FunctionMeasurement : public Measurement {
    private:
        std::string lastDataPoint{""};
        T (*measurementFunction)();
    public:
        FunctionMeasurement(std::string groupName, std::string measurementName, T (*function)()) : Measurement(groupName, measurementName) {
            measurementFunction = function;
        }

		void update() {
            std::string updatedData = std::to_string(measurementFunction());
            if (lastDataPoint.compare(updatedData) != 0) {
                lastDataPoint = updatedData;
                this->valueUpdated();
            }
        }

		std::string to_string() {
            this->valueUsed();
            return this->formatSingle(lastDataPoint);
        }
    };
    
} // namespace PT
