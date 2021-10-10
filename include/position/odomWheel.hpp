#pragma once

#include "api.h"

namespace Pronounce {
    /**
     * Abstract class for odometry wheels
     * 
     * @author ad101-lab
     */
    class OdomWheel
    {
    private:
        double mmPosition = 0;
        double lastPosition = 0;
    public:

        /**
         * Get the mm at the current moment
         */
        double getMM() {
            return this->mmPosition;
        }
        /**
         * Set the MM
         */
        void setMM(double mmPosition) {
            this->mmPosition = mmPosition;
        }

        double getChange() {
            double difference = this->getMM() - this->lastPosition;
            this->lastPosition = this->getMM();
            return difference;
        }

        void update();
        
        ~OdomWheel();
    };
} // namespace Pronounce
