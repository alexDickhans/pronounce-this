#pragma once

#include <cmath>
#include <string>
#include "utils.hpp"
#include "units/units.hpp"

namespace Pronounce {
    class Point {
    private:
        QLength x;
        QLength y;
    public:
        Point();
        Point(QLength x, QLength y);

        QLength distance(Point point) {
            QLength x = point.getX() - this->getX();
            QLength y = point.getY() - this->getY();

            QLength distance = sqrt(pow(x.getValue(), 2) + pow(y.getValue(), 2));

            return distance;
        }

        Point positionRelativeTo(Point point) {
            Point result = Point(point.getX() - this->getX(), point.getY() - this->getY());
            return result;
        }

		/**
		 * @brief Linear interpolation between two points
		 * 
		 * @param point1 The first point
		 * @param point2 The second point
		 * @param t The fraction to interpolate between the two points
		 * @return Point The result of the linear interpolation
		 */
		static Point lerp(Point point1, Point point2, double t) {
			Point result = Point(map(t, 0, 1, point1.getX().getValue(), point2.getX().getValue()), map(t, 0, 1, point1.getY().getValue(), point2.getY().getValue()));
			return result;
		}

		/**
		 * @brief Linear interpolation between two points
		 * 
		 * @param point2 The second point
		 * @param t The fraction to interpolate between the two points
		 * @return Point The result of the linear interpolation
		 */
		Point lerp(Point point2, double t) {
			return lerp(*this, point2, t);
		}


        QLength getX() {
            return this->x;
        }

        void setX(QLength x) {
            this->x = x;
        }

        QLength getY() {
            return this->y;
        }

        void setY(QLength y) {
            this->y = y;
        }

        void operator=(Point point) {
            this->setX(point.getX());
            this->setY(point.getY());
        }

		void operator=(Point* point) {
            this->setX(point->getX());
            this->setY(point->getY());
        }

        void add(Point point) {
            this->setX(this->getX() + point.getX());
            this->setY(this->getY() + point.getY());
        }

        void operator+=(Point point) {
            this->setX(this->getX() + point.getX());
            this->setY(this->getY() + point.getY());
        }

		void operator+=(Point* point) {
            this->setX(this->getX() + point->getX());
            this->setY(this->getY() + point->getY());
        }

        std::string to_string() {
            return "X: " + std::to_string(x.getValue()) + ", Y: " + std::to_string(y.getValue());
        }

        ~Point();
    };
} // namespace Pronounce
