#pragma once

#include "point.hpp"

namespace Pronounce {
    /**
     * @brief A class to handle vectors
     * 
     */
    class Vector {
    private:
        double magnitude;
        double angle;
    public:
        Vector();
        Vector(double magnitude, double angle);

        Point getCartesian();

        // Vector Math
        
        double dot(Vector x);
        Vector addition(Vector x);
        Vector scale(double scalar);

        ~Vector();
    };

    Vector::Vector() {
        this->magnitude = 0;
        this->angle = 0;
    }

    Vector::Vector(double magnitude, double angle) {
        this->magnitude = magnitude;
        this->angle = angle;
    }

    Point Vector::getCartesian() {
        Point result = Point();
        result.setX(magnitude * cos(angle));
        return result;
    }

    Vector::~Vector() {
    }
} // namespace Pronounce


