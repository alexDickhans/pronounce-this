#pragma once

#include "api.h"
#include <memory>
#include <string>
#include <stdexcept>

namespace Pronounce
{
    double toRadians(double degrees);
    double toDegrees(double radians);
    double mapFunc(double value, double start1, double stop1, double start2, double stop2);
    double signnum_c(double x);

    template<typename ... Args>
    std::string string_format(const std::string& format, Args ... args) {
        int size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
        if (size_s <= 0) { throw std::runtime_error("Error during formatting."); }
        auto size = static_cast<size_t>(size_s);
        auto buf = std::make_unique<char[]>(size);
        std::snprintf(buf.get(), size, format.c_str(), args ...);
        return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
    }
} // namespace Pronounce


