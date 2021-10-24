#pragma once

#include "api.h"

namespace Pronounce {
    /**
     * Abstract class for odometry wheels
     *
     * @author ad101-lab
     */
    class OdomWheel {
    private:
        double position = 0;
        double lastPosition = 0;
    public:

        /**
         * Get the mm at the current moment
         */
        double getPosition() {
            return this->position;
        }
        /**
         * Set the MM
         */
        void setPosition(double position) {
            this->position = position;
        }

        double getChange() {
            double difference = this->getPosition() - this->lastPosition;
            this->lastPosition = this->getPosition();
            return difference;
        }

        void update() {};

        ~OdomWheel();
    };
} // namespace Pronounce
