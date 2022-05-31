#pragma once

#include "api.h"
#include "utils/utils.hpp"
#include "utils/motorGroup.hpp"
#include "abstractDrivetrain.hpp"

namespace Pronounce {
    /**
     * Abstract class as a structure for different drivetrains assuming there are 4 motors
     */
    class Drivetrain : public AbstractDrivetrain {
    protected:

    public:
		Drivetrain();

		/**
         * Get average temperature of all the motors.
         */
        virtual double getTemp() { return 0; }

        /**
         * Get average speed of all the motors
         */
        virtual double getSpeed() { return 0; }

		~Drivetrain() {}
    };
} // namespace Pronounce

