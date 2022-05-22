#pragma once 

#include "abstractDrivetrain.hpp"
#include "utils/vector.hpp"

namespace Pronounce
{
    /**
     * Omnidirectional drive type, used for X-Drive/Mecanum drive
     */
    class AbstractHolonomicDrivetrain : public AbstractDrivetrain {
    private:
    public:
        AbstractHolonomicDrivetrain();

		virtual double getSpeed() { return 0; }

        virtual void setDriveVectorVelocity(Vector vector) {}
        virtual void setDriveVectorVelocity(Vector vector, double rotation) {}
        
        ~AbstractHolonomicDrivetrain();
    };
    
} // namespace Pronounce
