#include "api.h"

#include "odomWheel.hpp"
#include <iostream>

// TODO: add docstrings
// TODO: check code
// TODO: check units
// TODO: add comments

namespace Pronounce {
    class AdiOdomWheel : public OdomWheel {
    private:
        std::shared_ptr<pros::ADIEncoder> encoder;
    public:
        AdiOdomWheel();
        AdiOdomWheel(std::shared_ptr<pros::ADIEncoder> encoder);

        void update() {
            this->setPosition((encoder->get_value()/360.0) * this->getRadius().Convert(metre) * 1_pi * 2.0 * this->getTuningFactor());
        }

        void reset() {
            encoder->reset();
            this->setPosition(0.0);
            this->setLastPosition(0.0);
        }

        ~AdiOdomWheel();
    };    
} // namespace Pronounce
