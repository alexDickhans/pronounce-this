#pragma once

#include "units/units.hpp"

namespace Pronounce {

	/**
	 * @brief Orientation class that keeps track of different types of orientation objects
	 * 
	 */
	class Orientation {
	private:
		/**
		 * @brief The angle of the object
		 * 
		 */
		Angle angle;
	public:
		Orientation(Angle angle) : angle(angle) { }

		/**
		 * @brief Update the values
		 * 
		 */
		virtual void update() {}

		/**
		 * @brief Set the Angle object
		 * 
		 * @param angle New angle
		 */
		void setAngle(Angle angle) {
			this->angle = angle;
		}

		/**
		 * @brief Get the Angle object
		 * 
		 * @return Angle The new angle
		 */
		Angle getAngle() {
			return angle;
		}

		/**
		 * @brief Reset the values
		 * 
		 */
		virtual void reset() {
			angle = 0.0;
		}

		~Orientation() {}
	};
} // namespace Pronounce
