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
        double width;
		double length;
		double wheelAngle;
    public:
        AbstractHolonomicDrivetrain();
        AbstractHolonomicDrivetrain(double trackWidth, double wheelAngle);

		virtual double getSpeed() {}

		void setWidth(double width) {
			this->width = width;
		}

		double getWidth() {
			return width;
		}

		void setLength(double length) {
			this->length = length;
		}

		double getLength() {
			return this->length;
		}

		void setWheelAngle(double wheelAngle) {
			this->wheelAngle = wheelAngle;
		}

		double getWheelangle() {
			return wheelAngle;
		}

        virtual void setDriveVectorVelocity(Vector vector) {}
        virtual void setDriveVectorVelocity(Vector vector, double rotation) {}
        
        ~AbstractHolonomicDrivetrain();
    };
    
} // namespace Pronounce
