#include "api.h"

#include "odomWheel.hpp"

namespace Pronounce {
    class  AdiOdomWheel : public OdomWheel {
    private:
        pros::ADIEncoder* encoder;
    public:
        AdiOdomWheel();
        AdiOdomWheel(pros::ADIEncoder* encoder);

        void update() {
            this->setPosition((encoder->get_value()/360) * this->getRadius() * M_PI * 2.0 * this->getTuningFactor());
        }

        ~AdiOdomWheel();
    };    
} // namespace Pronounce
