#pragma once

#include "abstractDrivetrain.hpp"
#include "utils/position.hpp"

namespace Pronounce {
	/**
	 * @brief A class for all simulator drivetrains
	 * 
	 */
	class SimDrivetrain : public AbstractDrivetrain {
	private:
		Position* position;
		double resetOrientation = 0.0;
	public:
		SimDrivetrain();
		SimDrivetrain(Position* position);
		
		Position* getPosition() {
			return position;
		}

		void setPosition(Position* position) {
			this->position = position;
		}

		double getResetOrientation() {
			return resetOrientation;
		}

		void setResetOrientation(double resetPosition) {
			this->resetOrientation = resetOrientation;
		}

		void reset(Position* position) {
			this->position->operator=(position);
			this->resetOrientation = position->getTheta();
		}

		virtual void update() {}

		~SimDrivetrain();
	};	
} // namespace Pronounce
