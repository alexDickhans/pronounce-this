#pragma once

namespace Pronounce {
    class Vector {
    private:
        double magnitude;
        double angle;

    public:
        Vector();
        Vector(double angle);
        Vector(double magnitude, double angle);
        Vector()
        ~Vector();
    };
    
    Vector::Vector(/* args */) {
    }
    
    Vector::~Vector() {
    }
    
} // namespace Pronounce
