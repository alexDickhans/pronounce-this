#include "vector.hpp"

namespace Pronounce {
    Vector::Vector() {
        this->magnitude = 0;
        this->angle = 0;
    }

    Vector::Vector(QLength magnitude, Angle angle) {
		if (magnitude < (QLength) 0.0) {
			this->magnitude = -magnitude.getValue();
			this->angle = angle.getValue() + M_PI;
		}
        this->magnitude = magnitude;
        this->angle = angle;
    }

    Vector::Vector(Point* point) {
        this->magnitude = point->distance(Point()).getValue();
        this->angle = atan2(point->getY().getValue(), point->getX().getValue());
    }

    Vector::Vector(Point* point1, Point* point2) {
        this->magnitude = point1->distance(*point2).getValue();
        this->angle = atan2((point2->getY() - point1->getY()).getValue(), (point2->getX() - point1->getX()).getValue());
    }

    Point Vector::getCartesian() {
        Point result = Point();
        result.setX(magnitude.getValue() * cos(angle));
        result.setY(magnitude.getValue() * sin(angle));
        return result;
    }

    double Vector::dot(Vector x) {
        Point thisPoint = this->getCartesian();
        Point xPoint = x.getCartesian();

        double result = (thisPoint.getX() * xPoint.getX() +
            thisPoint.getY() * xPoint.getY()).getValue();

        return result;
    }

    Vector Vector::addition(Vector x) {
        Point thisPoint = this->getCartesian();
        Point xPoint = this->getCartesian();

        Point resultPoint = Point(thisPoint.getX() + xPoint.getX(), thisPoint.getY() + xPoint.getY());
        Vector result = Vector(&resultPoint);

        return result;
    }

    Vector Vector::scale(double scalar) {
        Vector vector = Vector(*this);

        vector.setMagnitude(vector.getMagnitude() * scalar);

        return vector;
    }

    Vector::~Vector() {
    }
} // namespace Pronounce