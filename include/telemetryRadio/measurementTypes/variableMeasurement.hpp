#pragma once

#include "telemetryRadio/measurementTypes/functionMeasurement.hpp"
#include <string>

namespace PT {
    template <typename T>
    class VariableMeasurement {
    private:
        T* variable;

        std::string lastDataPoint;
    public:
        VariableMeasurement(T* variable) {
            this->variable = variable;
        }

        void update() {
            std::string updatedData = std::to_string(variable);
            if (lastDataPoint != updatedData) {
                lastDataPoint = updatedData;
                this->valueUpdated();
            }
        }

		std::string to_string() {
            this->valueUsed();
            return this->formatSingle(lastDataPoint);
        }

        ~VariableMeasurement() {}
    };
} // namespace PT
