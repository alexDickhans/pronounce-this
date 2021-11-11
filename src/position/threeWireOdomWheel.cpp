#include "threeWireOdomWheel.hpp"

namespace Pronounce {
    AdiOdomWheel::AdiOdomWheel() {
    }

    AdiOdomWheel::AdiOdomWheel(pros::ADIEncoder* encoder) {
        this->encoder = encoder;
    }
    
    AdiOdomWheel::~AdiOdomWheel()
    {
    }
} // namespace Pronounce
