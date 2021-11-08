#include "vector.hpp"

namespace Pronounce {
    Vector::Vector() {
        this->magnitude = 0;
        this->angle = 0;
    }

    Vector::Vector(double magnitude, double angle) {
        this->magnitude = magnitude;
        this->angle = angle;
    }

    Vector::Vector(Point point) {
        this->magnitude = point.distance(Point(0, 0));
        this->angle = acos(point.getX() / magnitude);
    }

    Point Vector::getCartesian() {
        Point result = Point();
        result.setX(magnitude * cos(angle));
        return result;
    }

    double Vector::dot(Vector x) {
        Point thisPoint = this->getCartesian();
        Point xPoint = this->getCartesian();

        double result = thisPoint.getX() * xPoint.getX() +
            thisPoint.getY() * xPoint.getY();
    }

    Vector Vector::addition(Vector x) {
        Point thisPoint = this->getCartesian();
        Point xPoint = this->getCartesian();

        Point resultPoint = Point(thisPoint.getX() + xPoint.getX(), thisPoint.getY() + xPoint.getY());
        Vector result = Vector(resultPoint);
    }

    Vector Vector::scale(double scalar) {
        Vector vector = *this;

        vector.setMagnitude(vector.getMagnitude() * scalar);

        return vector;
    }

    Vector::~Vector() {
    }
} // namespace Pronounce
