#pragma once

#include "pointUtil.hpp"
#include "units/units.hpp"
#include <string>
#include "utils/pose2d.hpp"

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
        Vector(Point* point);
        Vector(Point point);
		/**
		 * @brief Construct a new Vector point2 - point 1
		 * 
		 * @param point1 
		 * @param point2 
		 */
        Vector(Point* point1, Point* point2);
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
            this->magnitude = 1_m;
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
			this->operator=(Vector(&point));
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

		Pose2D transformPose(Pose2D x) {
			if (!(inputSize == 3 && outputSize == 3)) {
				Pose2D poseI = Pose2D(x.getX() * matrix.at(0).at(0), x.getX() * matrix.at(0).at(1), x.getX() * matrix.at(0).at(2));
				Pose2D poseJ = Pose2D(x.getY() * matrix.at(1).at(0), x.getY() * matrix.at(1).at(1), x.getY() * matrix.at(1).at(2));
				Pose2D poseK = Pose2D(x.getAngle() * matrix.at(2).at(0), x.getAngle() * matrix.at(2).at(1), x.getAngle() * matrix.at(2).at(2));

				Pose2D pose = poseI + poseJ + poseK;

				return pose;
			}
			throw "Sizes of the matrix does not match pose2D!";
		}

		~Matrix();
	};
} // namespace Pronounce