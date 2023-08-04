#pragma once

#include "odomWheel.hpp"
#include "units/units.hpp"
#include <vector>

// TODO: Change folder name to be something more desciptive
// TODO: Make sure docstrings are up to date
// TODO: Add comments

namespace Pronounce {

    /**
     * @brief Class to average multiple odom wheels, useful for tank drives and similar base designs.
     *
     * @authors Alex(ad101-lab)
     */
    class AvgOdom : public OdomWheel {
    private:
        std::vector<OdomWheel> odomWheels;
    public:
        AvgOdom();
		AvgOdom(std::vector<OdomWheel> odomWheels);

        /**
         * Get the distance at the current moment
         */
        QLength getPosition() {
            QLength total = 0.0;
            for (int i = 0; i < odomWheels.size(); i++) {
                total += odomWheels.at(i).getPosition();
            }
            return total / odomWheels.size();
        }
        /**
         * Set the distance
         */
        void setPosition(double position) {
			for (int i = 0; i < odomWheels.size(); i++) {
                odomWheels.at(i).setPosition(position);
            }
        }

        QLength getChange() {
            QLength total = 0.0;
			for (int i = 0; i < odomWheels.size(); i++) {
                total += odomWheels.at(i).getChange();
            }
            return total / odomWheels.size();
        }

        void update() {
			for (int i = 0; i < odomWheels.size(); i++) {
                odomWheels.at(i).update();
            }
        }

		void reset() {
			for (int i = 0; i < odomWheels.size(); i++) {
                odomWheels.at(i).reset();
            }
		}

        ~AvgOdom();
    };
} // namespace Pronounce


