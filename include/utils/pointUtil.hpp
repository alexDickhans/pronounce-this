#pragma once

#include <cmath>
#include <string>

namespace Pronounce {
    class Point {
    private:
        double x;
        double y;
    public:
        Point();
        Point(double x, double y);

        double distance(Point point) {
            double x = point.getX() - this->getX();
            double y = point.getY() - this->getY();

            double distance = sqrt(pow(x, 2) + pow(y, 2));

            return distance;
        }

        Point positionRelativeTo(Point point) {
            Point result = Point(point.getX() - this->getX(), point.getY() - this->getY());
            return result;
        }

        double getX() {
            return this->x;
        }

        void setX(double x) {
            this->x = x;
        }

        double getY() {
            return this->y;
        }

        void setY(double y) {
            this->y = y;
        }

        void operator=(Point point) {
            this->setX(point.getX());
            this->setY(point.getY());
        }

        void add(Point point) {
            this->setX(this->getX() + point.getX());
            this->setY(this->getY() + point.getY());
        }

        void operator+=(Point point) {
            this->setX(this->getX() + point.getX());
            this->setY(this->getY() + point.getY());
        }

        std::string to_string() {
            return "X: " + std::to_string(x) + ", Y: " + std::to_string(y);
        }

        ~Point();
    };
} // namespace Pronounce
