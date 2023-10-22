#pragma once

#include <iostream>

#include "transmitter.hpp"
#include "encoding.hpp"

namespace PT {
    class USB : public Transmitter {
    private:
        Encoding* encoder;
    public:
        USB(Encoding* encoder = new PassThroughEncoding());

        void transmit(std::string data);

        ~USB();
    };
} // namespace PT
