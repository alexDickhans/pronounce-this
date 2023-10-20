#pragma once

#include "point.hpp"
#include "../../units/units.hpp"
#include <string>

// TODO: add docstrings
// TODO: add comments

namespace PathPlanner {
	/**
	 * @brief A class to handle vectors
	 *
	 * @author Alex(ad101-lab)
	 */
	class Vector {
	private:
		QLength magnitude;
		Angle angle;
	public:
		Vector() {
			this->magnitude = 0.0;
			this->angle = 0.0;
		}
		Vector(Point point) {
			this->magnitude = point.distance(Point()).getValue();
			this->angle = atan2(point.getY().getValue(), point.getX().getValue());
		}
		Vector(Point point1, Point point2) {
			this->magnitude = point1.distance(point2).getValue();
			this->angle = atan2((point2.getY() - point1.getY()).getValue(), (point2.getX() - point1.getX()).getValue());
		}
		Vector(QLength magnitude, Angle angle) {
			if (magnitude < (QLength) 0.0) {
				this->magnitude = -magnitude.getValue();
				this->angle = angle.getValue() + M_PI;
			}
			this->magnitude = magnitude;
			this->angle = angle;
		}

		void operator=(Vector vector) {
			this->setAngle(vector.getAngle());
			this->setMagnitude(vector.getMagnitude());
		}

		/**
		 * @brief Get the vector in the cartesian reference frame
		 *
		 * @return Point
		 */
		Point getCartesian() {
			Point result = Point();
			result.setX(magnitude.getValue() * cos(angle));
			result.setY(magnitude.getValue() * sin(angle));
			return result;
		}

		// Vector Math

		/**
		 * @brief Normalize the vector
		 *
		 */
		void normalize() {
			this->magnitude = 1.0;
		}

		/**
		 * @brief Get the dot product of this vector and x
		 *
		 * @param x Other vector
		 * @return double The dot product of this vector and another vector
		 */
		double dot(Vector x) {
			Point thisPoint = this->getCartesian();
			Point xPoint = x.getCartesian();

			double result = (thisPoint.getX() * xPoint.getX() +
				thisPoint.getY() * xPoint.getY()).getValue();

			return result;
		}

		/**
		 * @brief Get the sum of this vector and x
		 *
		 * @param x
		 * @return Vector
		 */
		Vector addition(Vector x){
			Point thisPoint = this->getCartesian();
			Point xPoint = this->getCartesian();

			Point resultPoint = Point(thisPoint.getX() + xPoint.getX(), thisPoint.getY() + xPoint.getY());
			Vector result = Vector(resultPoint);

			return result;
		}

		/**
		 * @brief Scale this vector by a scalar
		 *
		 * @param scalar The scalar double
		 * @return Vector The resulting vector
		 */
		Vector scale(double scalar){
			Vector vector = Vector(*this);

			vector.setMagnitude(vector.getMagnitude().getValue() * scalar);

			return vector;
		}

		/**
		 * @brief Get the Magnitude
		 *
		 * @return double Magnitude
		 */
		QLength getMagnitude() {
			return magnitude;
		}

		/**
		 * @brief Set the magnitude
		 *
		 * @param magnitude New magnitude value
		 */
		void setMagnitude(QLength magnitude) {
			this->magnitude = magnitude;
		}

		/**double
		 * @brief Get the angle
		 *
		 * @return double angle
		 */
		Angle getAngle() {
			return angle;
		}

		/**
		 * @brief Set the angle
		 *
		 * @param magnitude New angle value
		 */
		void setAngle(Angle angle) {
			this->angle = angle;
		}

		std::string to_string() {
			return "Magnitude: " + std::to_string(magnitude.Convert(inch)) + " angle: " + std::to_string(angle.Convert(degree));
		}

		void operator=(Point point) {
			this->operator=(Vector(point));
		}

		~Vector() {
		}
	};
} // namespace Pronounce