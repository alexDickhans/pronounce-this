#pragma once

#include "abstractDrivetrain.hpp"
#include "utils/pose2d.hpp"

namespace Pronounce {
	/**
	 * @brief A class for all simulator drivetrains
	 * 
	 */
	class SimDrivetrain : public AbstractDrivetrain {
	private:
		Pose2D* position;
		double resetOrientation = 0.0;
	public:
		SimDrivetrain();
		SimDrivetrain(Pose2D* position);
		
		Pose2D* getPosition() {
			return position;
		}

		void setPosition(Pose2D* position) {
			this->position = position;
		}

		double getResetOrientation() {
			return resetOrientation;
		}

		void setResetOrientation(double resetPosition) {
			this->resetOrientation = resetOrientation;
		}

		void reset(Pose2D* position) {
			this->position->operator=(position);
			this->resetOrientation = position->getAngle();
		}

		virtual void update() {}

		~SimDrivetrain();
	};	
} // namespace Pronounce
