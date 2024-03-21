#pragma once

#include "../../units/units.hpp"
#include "json/asset.hpp"
#include "json/json.hpp"
#include <cmath>
#include <string>

namespace PathPlanner {

	/**
	 * @brief Point class for a 2d point on the field
	 * 
	 * @authors Alex Dickhans(alexDickhans)
	 */
    class Point {
    private:
		/**
		 * @brief X and y values
		 * 
		 */
        QLength x, y;
    public:
		/**
		 * @brief Construct a new Point with all values set to 0
		 * 
		 */
        Point() {
			this->x = 0.0;
			this->y = 0.0;
		}

		/**
		 * @brief Construct a new Point with x and y set
		 * 
		 * @param x X value
		 * @param y Y value
		 */
        Point(QLength x, QLength y) {
			this->x = x;
			this->y = y;
		}

		Point(Json jsonObject) {
			this->x = jsonObject["x"].number_value();
			this->y = jsonObject["y"].number_value();
		}

		/**
		 * @brief Get the distance from another point
		 * 
		 * @param point Other point
		 * @return QLength The distance between the 2 points
		 */
        QLength distance(Point point) {
			// Calculate the distance in x and y
            QLength x = point.getX() - this->getX();
            QLength y = point.getY() - this->getY();
			
			// Calculate distance with typesafe function
            QLength distance = Qsqrt(Qsq(x) + Qsq(y));
			
			// Return the value
            return distance;
        }

		/**
		 * @brief this - point
		 * 
		 * @param point The subtration value
		 * @return Point The result
		 */
        Point positionRelativeTo(Point point) {
            Point result = Point(point.getX() - this->getX(), point.getY() - this->getY());
            return result;
        }

		/**
		 * @brief Get the x value
		 * 
		 * @return QLength The current x value
		 */
        QLength getX() {
            return this->x;
        }

		/**
		 * @brief Set the x value
		 * 
		 * @param x New x value
		 */
        void setX(QLength x) {
            this->x = x;
        }

		/**
		 * @brief Get the y value
		 * 
		 * @return QLength The current y value
		 */
        QLength getY() {
            return this->y;
        }
		
		/**
		 * @brief Set the y value
		 * 
		 * @param y New y value
		 */
        void setY(QLength y) {
            this->y = y;
        }

		/**
		 * @brief Set the point with an equals operator 
		 * 
		 * @param point The new point
		 */
        void operator=(Point point) {
            this->setX(point.getX());
            this->setY(point.getY());
        }

		/**
		 * @brief Set the point with an equals operator 
		 * 
		 * @param point The new point
		 */
		void operator=(Point* point) {
            this->setX(point->getX());
            this->setY(point->getY());
        }

		/**
		 * @brief Add points
		 * 
		 * @param point The point to add
		 */
        void add(Point point) {
            this->setX(this->getX() + point.getX());
            this->setY(this->getY() + point.getY());
        }

		/**
		 * @brief Add values using +=
		 * 
		 * @param point The point to add
		 */
        void operator+=(Point point) {
            this->setX(this->getX() + point.getX());
            this->setY(this->getY() + point.getY());
        }

		/**
		 * @brief Add values using +=
		 * 
		 * @param point The point to add
		 */
		void operator+=(Point* point) {
            this->setX(this->getX() + point->getX());
            this->setY(this->getY() + point->getY());
        }

		/**
		 * @brief Add values using +
		 * 
		 * @param point The point to add
		 */
		Point operator+(Point b) {
			return Point(this->getX() + b.getX(), this->getY() + b.getY());
		}

		/**
		 * @brief Create a formatted string to use for debugging
		 * 
		 * @return std::string The formatted string with all the values
		 */
        std::string to_string() {
            return "X: " + std::to_string(x.Convert(inch)) + ", Y: " + std::to_string(y.Convert(inch));
        }
    };
} // namespace Pronounce
