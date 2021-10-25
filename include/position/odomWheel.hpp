#pragma once

#include "api.h"

namespace Pronounce {
    /**
     * Abstract class for odometry wheels
     *
     * @author Alex(ad101-lab)
     */
    class OdomWheel {
    private:
        double position = 0;
        double lastPosition = 0;
    public:
        OdomWheel();

        /**
         * Get the mm at the current moment
         */
        virtual double getPosition() {
            return this->position;
        }
        /**
         * Set the MM
         */
        virtual void setPosition(double position) {
            this->position = position;
        }

        virtual double getChange() {
            double difference = this->getPosition() - this->lastPosition;
            this->lastPosition = this->getPosition();
            return difference;
        }

        virtual void update() {};

        ~OdomWheel();
    };
} // namespace Pronounce
