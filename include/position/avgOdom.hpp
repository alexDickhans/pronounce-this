#pragma once

#include "odomWheel.hpp"

namespace Pronounce {

    /**
     * @brief Class to average multiple odom wheels, useful for tank drives and similar base designs.
     * 
     * @tparam I Wheel count
     * 
     * @authors Alex(ad101-lab)
     */
    template<int I>
    class AvgOdom : OdomWheel {
    private:
        OdomWheel odomWheels[I];
    public:
        AvgOdom();
        AvgOdom(OdomWheel odomWheels[I]);

        /**
         * Get the distance at the current moment
         */
        double getPosition() {
            double total = 0;
            for (int i = 0; i < I; i++) {
                total += odomWheels[i].getPosition();
            }
            return total / I;
        }
        /**
         * Set the distance
         */
        void setPosition(double position) {
            for (int i = 0; i < I; i++) {
                odomWheels[i].setPosition(position);
            }
        }

        double getChange() {
            double total = 0;
            for (int i = 0; i < I; i++) {
                total += odomWheels[i].getChange();
            }
            return total / I;
        }

        void update() {
            for (int i = 0; i < I; i++) {
                odomWheels[i].update();
            }
        }

        ~AvgOdom();
    };

    template<int I>
    AvgOdom<I>::AvgOdom() {
    }

    template<int I>
    AvgOdom<I>::AvgOdom(OdomWheel odomWheels[I]) {
    }

    template<int I>
    AvgOdom<I>::~AvgOdom() {
    }
} // namespace Pronounce


