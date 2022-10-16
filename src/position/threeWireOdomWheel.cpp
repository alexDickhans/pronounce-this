#include "threeWireOdomWheel.hpp"

namespace Pronounce {
    AdiOdomWheel::AdiOdomWheel() {
    }

    AdiOdomWheel::AdiOdomWheel(std::shared_ptr<pros::ADIEncoder> encoder) {
        this->encoder = encoder;
    }
    
    AdiOdomWheel::~AdiOdomWheel()
    {
    }
} // namespace Pronounce
