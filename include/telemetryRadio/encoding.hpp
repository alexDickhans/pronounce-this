#pragma once

#include <vector>
#include <stdint.h>
#include <string>

namespace PT {
    class Encoding {
    private:
        
    public:
        Encoding(/* args */);

        virtual std::vector<uint8_t> encode(std::vector<uint8_t> data) { return std::vector<uint8_t>(); }

        virtual std::vector<uint8_t> decode(std::vector<uint8_t> rawData) { return std::vector<uint8_t>(); }

        static std::vector<uint8_t> reinterpretString(std::string data);
         
        ~Encoding() {}
    };

    class COBSEncoding : public Encoding {
    private:

    public:
        COBSEncoding();

        std::vector<uint8_t> encode(std::vector<uint8_t> data);

        /**
         *
        */
        std::vector<uint8_t> decode(std::vector<uint8_t> rawData);

        ~COBSEncoding();
    };
    
    class PassThroughEncoding : public Encoding {
    private:
        
    public:
        PassThroughEncoding();

        virtual std::vector<uint8_t> encode(std::vector<uint8_t> data) { return data; }

        virtual std::vector<uint8_t> decode(std::vector<uint8_t> rawData) { return rawData; }

        ~PassThroughEncoding();
    };
} // namespace PR
