#include "vector.hpp"

namespace Pronounce {
    Vector::Vector() {
        this->magnitude = 0;
        this->angle = 0;
    }

    Vector::Vector(double magnitude, double angle) {
		if (magnitude < 0) {
			this->magnitude = -magnitude;
			this->angle = angle + M_PI;
		}
        this->magnitude = magnitude;
        this->angle = angle;
    }

    Vector::Vector(Point* point) {
        this->magnitude = point->distance(Point());
        this->angle = atan2(point->getY(), point->getX());
    }

    Vector::Vector(Point* point1, Point* point2) {
        this->magnitude = point1->distance(*point2);
        this->angle = atan2(point2->getY() - point1->getY(), point2->getX() - point1->getX());
    }

    Point Vector::getCartesian() {
        Point result = Point();
        result.setX(magnitude * cos(angle));
        result.setY(magnitude * sin(angle));
        return result;
    }

    double Vector::dot(Vector x) {
        Point thisPoint = this->getCartesian();
        Point xPoint = x.getCartesian();

        double result = thisPoint.getX() * xPoint.getX() +
            thisPoint.getY() * xPoint.getY();

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