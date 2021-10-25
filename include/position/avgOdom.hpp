#pragma once

#include "odomWheel.hpp"
#include <list>

namespace Pronounce {

    /**
     * @brief Class to average multiple odom wheels, useful for tank drives and similar base designs.
     *
     * @tparam I Wheel count
     *
     * @authors Alex(ad101-lab)
     */
    class AvgOdom : public OdomWheel {
    private:
        std::list<OdomWheel> odomWheels;
    public:
        AvgOdom();
        AvgOdom(OdomWheel odomWheels[]);
        AvgOdom(std::list<OdomWheel> odomWheels);
        AvgOdom(std::list<OdomWheel>* odomWheels);

        /**
         * Get the distance at the current moment
         */
        double getPosition() {
            double total = 0;
            for (std::list<OdomWheel>::iterator it = odomWheels.begin(); it != odomWheels.end(); it++) {
                total += it->getPosition();
            }
            return total / odomWheels.size();
        }
        /**
         * Set the distance
         */
        void setPosition(double position) {
            for (std::list<OdomWheel>::iterator it = odomWheels.begin(); it != odomWheels.end(); it++) {
                it->setPosition(position);
            }
        }

        double getChange() {

            double total = 0;
            for (std::list<OdomWheel>::iterator it = odomWheels.begin(); it != odomWheels.end(); it++) {
                total += it->getChange();
            }
            return total / odomWheels.size();
        }

        void update() {
            for (std::list<OdomWheel>::iterator it = odomWheels.begin(); it != odomWheels.end(); it++) {
                it->update();
            }
        }

        ~AvgOdom();
    };

    AvgOdom::AvgOdom() {
    }

    AvgOdom::AvgOdom(std::list<OdomWheel> odomWheels) {
        this->odomWheels = odomWheels;
    }

    AvgOdom::AvgOdom(std::list<OdomWheel>* odomWheels) {
        this->odomWheels = *odomWheels;
    }

    AvgOdom::~AvgOdom() {
    }
} // namespace Pronounce


