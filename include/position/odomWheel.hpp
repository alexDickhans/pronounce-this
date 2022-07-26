#pragma once

#include "api.h"
#include "units/units.hpp"

namespace Pronounce {
    /**
     * Abstract class for odometry wheels
     *
     * @author Alex(ad101-lab)
     */
    class OdomWheel {
    private:
        QLength position = 0;
        QLength lastPosition = 0;

        QLength radius;

        double tuningFactor = 1.0;
    public:
        OdomWheel();
        OdomWheel(double radius);

        virtual void reset() {
            position = 0;
            lastPosition = 0;
        }
        
        QLength getRadius() {
            return this->radius;
        }

        void setRadius(QLength radius) {
            this->radius = radius;
        }

        double getTuningFactor() {
            return this->tuningFactor;
        }

        void setTuningFactor(double tuningFactor) {
            this->tuningFactor = tuningFactor;
        }

        /**
         * Get the position at the current moment
         */
        virtual QLength getPosition() {
            return this->position;
        }
        /**
         * Set the position
         */
        virtual void setPosition(QLength position) {
            this->position = position;
        }

        /**
         * Get the last position
         */
        QLength getLastPosition() {
            return this->lastPosition;
        }

        /**
         * Set the last position
         */
        void setLastPosition(QLength lastPosition) {
            this->lastPosition = lastPosition;
        }

        QLength getChange() {
            QLength difference = this->getPosition() - this->lastPosition;
            this->lastPosition = this->getPosition();
            return difference;
        }

        virtual void update() {};

        ~OdomWheel();
    };
} // namespace Pronounce
