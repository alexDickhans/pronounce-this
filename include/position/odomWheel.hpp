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

        double radius;

        double tuningFactor = 1.0;
    public:
        OdomWheel();
        OdomWheel(double radius);

        void reset() {
            position = 0;
            lastPosition = 0;
        }
        
        double getRadius() {
            return this->radius;
        }

        void setRadius(double radius) {
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
        virtual double getPosition() {
            return this->position;
        }
        /**
         * Set the position
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
