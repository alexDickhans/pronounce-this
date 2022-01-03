#pragma once

#include "abstractDrivetrain.hpp"
#include "utils/position.hpp"

namespace Pronounce {
	/**
	 * @brief A class for all simulator drivetrains
	 * 
	 */
	class SimDrivetrain {
	private:
		Position* position;
	public:
		SimDrivetrain();
		SimDrivetrain(Position* position);
		
		Position* getPosition() {
			return position;
		}

		void setPosition(Position* position) {
			this->position = position;
		}

		virtual void update() {}

		~SimDrivetrain();
	};	
} // namespace Pronounce
