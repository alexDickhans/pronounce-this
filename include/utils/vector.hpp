#pragma once

#include "pointUtil.hpp"
#include <string>

namespace Pronounce {
    /**
     * @brief A class to handle vectors
     *
     * @author Alex(ad101-lab)
     */
    class Vector {
    private:
        double magnitude;
        double angle;
    public:
        Vector();
        Vector(Point* point);
        Vector(Point* point1, Point* point2);
        Vector(double magnitude, double angle);

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
            this->magnitude = 1;
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
        double getMagnitude() {
            return magnitude;
        }

        /**
         * @brief Set the magnitude
         *
         * @param magnitude New magnitude value
         */
        void setMagnitude(double magnitude) {
            this->magnitude = magnitude;
        }

        /**
         * @brief Get the angle
         *
         * @return double angle
         */
        double getAngle() {
            return angle;
        }

        /**
         * @brief Set the angle
         *
         * @param magnitude New angle value
         */
        void setAngle(double angle) {
            this->angle = angle;
        }

        void rotate(double angle) {
            this->angle += angle;
        }

        std::string to_string() {
            return "Magnitude: " + std::to_string(magnitude) + " angle: " + std::to_string(angle);
        }

        ~Vector();
    };
} // namespace Pronounce