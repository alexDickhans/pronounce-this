#pragma once

#include "units/units.hpp"
#include "orientation.hpp"
#include <vector>

namespace Pronounce {

	/**
	 * @brief Average multiple orientation values
	 * 
	 * @authors Alex Dickhans (alexDickhans) 
	 */
	class AvgOrientation : public Orientation {
	private:
		/**
		 * @brief List of orientations
		 * 
		 */
		std::vector<Orientation*> orientations;
	public:
		/**
		 * @brief Construct a new Avg Orientation object
		 * 
		 */
		AvgOrientation() : Orientation(0.0) {
			orientations = std::vector<Orientation*>();
		}

		/**
		 * @brief Update the values in the orientation values
		 * 
		 */
		void update() {
			Angle total = 0.0;

			for (int i = 0; i < orientations.size(); i ++) {
				orientations.at(i)->update();
				total += orientations.at(i)->getAngle();
			}

			this->setAngle(total/orientations.size());
		}

		/**
		 * @brief Reset all the values
		 * 
		 */
		void reset() {
			for (int i = 0; i < orientations.size(); i++) {
				orientations.at(i)->reset();
			}
		}

		/**
		 * @brief Add a orientation value to the list
		 * 
		 * @param orientation The orientation pointer
		 */
		void addOrientation(Orientation* orientation) {
			this->orientations.emplace_back(orientation);
		}

		~AvgOrientation() {}
	};
} // namespace Pronounce
