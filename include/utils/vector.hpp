#pragma once

#include "point.hpp"
#include "units/units.hpp"
#include <string>

// TODO: add docstrings
// TODO: add comments

namespace Pronounce {
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
        Vector();
        Vector(Point point);
		Vector(Point point1, Point point2);
        Vector(QLength magnitude, Angle angle);

        void operator=(Vector vector) {
            this->setAngle(vector.getAngle());
            this->setMagnitude(vector.getMagnitude());
        }

        /**
         * @brief Get the vector in the cartesian reference frame
         *
         * @return Point
         */
        Point getCartesian();

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
        double dot(Vector x);

        /**
         * @brief Get the sum of this vector and x
         *
         * @param x
         * @return Vector
         */
        Vector addition(Vector x);

        /**
         * @brief Scale this vector by a scalar
         *
         * @param scalar The scalar double
         * @return Vector The resulting vector
         */
        Vector scale(double scalar);

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

        void rotate(Angle angle) {
            this->angle += angleDifference(angle.getValue(), 0);
			
        }

        std::string to_string() {
            return "Magnitude: " + std::to_string(magnitude.Convert(inch)) + " angle: " + std::to_string(angle.Convert(degree));
        }

		void operator=(Point point) {
			this->operator=(Vector(point));
		}

        ~Vector();
    };

	template<uint32_t inputSize, uint32_t outputSize>
	class Matrix {
	private:
		std::array<std::array<double, inputSize>, outputSize> matrix;
	public:
		Matrix();
		Matrix(std::initializer_list<std::initializer_list<double>> matrix);
		Matrix(std::array<std::array<double, inputSize>, outputSize> matrix);

		/**
		 * @brief This vector * x
		 * 
		 * @param x The other matrix to multiply
		 * @return Matrix<inputSize, outputSize> matrix of the same size
		 */
		Matrix<inputSize, outputSize> multiply(Matrix<inputSize, outputSize> x);

		Vector transformVector(Vector x);

		~Matrix();
	};
} // namespace Pronounce