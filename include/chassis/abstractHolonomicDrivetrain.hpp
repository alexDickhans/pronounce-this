#pragma once 

#include "abstractDrivetrain.hpp"
#include "utils/vector.hpp"

namespace Pronounce
{
    /**
     * Omnidirectional drive type, used for X-Drive/Mecanum drive
	 * 
	 * @authors Alex Dickhans(ad101-lab)
     */
    class AbstractHolonomicDrivetrain : public AbstractDrivetrain {
    private:
    public:
        AbstractHolonomicDrivetrain() {}

		/**
		 * @brief Set the drive vector relative to the robot using velocity
		 * 
		 * @param vector The vector to drive to, normalized
		 */
        virtual void setDriveVectorVelocity(Vector vector) {}

		/**
		 * @brief Set the drive vector relative to the robot using velocity and rotation
		 * 
		 * @param vector The vector to drive to, normalized 
		 * @param rotation 
		 */
        virtual void setDriveVectorVelocity(Vector vector, double rotation) {}
        
        ~AbstractHolonomicDrivetrain() {}
    };
    
} // namespace Pronounce
