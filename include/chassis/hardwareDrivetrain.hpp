#pragma once

#include "api.h"
#include "utils/utils.hpp"
#include "abstractDrivetrain.hpp"

namespace Pronounce {
    /**
     * Abstract class as a structure for different drivetrains assuming there are 4 motors
     */
    class HardwareDrivetrain {
    protected:

    public:
		HardwareDrivetrain();

		/**
         * Get average temperature of all the motors.
         */
        virtual double getTemp() { return 0; }

		~HardwareDrivetrain() {}
    };
} // namespace Pronounce

