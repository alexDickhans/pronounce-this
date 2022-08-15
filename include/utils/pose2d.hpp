#pragma once

#include "pointUtil.hpp"
#include <string>
#include "units/units.hpp"

namespace Pronounce {
    /**
     * A class to keep track of pose on the field
	 * 
	 * @authors Alex Dickhans (alexDickhans)
     */
    class Pose2D : public Point {
    private:
		/**
		 * @brief Angle of the pose
		 * 
		 */
        Angle angle = 0.0;
    public:
		/**
		 * @brief Construct a new Pose 2 D object with all values set to 0
		 * 
		 */
        Pose2D();

		/**
		 * @brief Construct a new Pose 2 D object
		 * 
		 * @param angle 
		 */
        Pose2D(Angle angle);

		/**
		 * @brief Construct a new Pose 2 D object with double x and y objects(in meters)
		 * 
		 * @param x X position in meters
		 * @param y Y position in meters
		 */
        Pose2D(double x, double y);

		/**
		 * @brief Construct a new Pose 2 D object with x and y set to length objects
		 * 
		 * @param x X position
		 * @param y Y position
		 */
        Pose2D(QLength x, QLength y) : Point(x, y) {}

		/**
		 * @brief Construct a new Pose 2 D object with length and angle objects
		 * 
		 * @param x X position
		 * @param y Y position
		 * @param angle angle
		 */
        Pose2D(QLength x, QLength y, Angle angle) : Point(x, y), angle(angle) {}

		/**
		 * @brief Get a formatted string of the object
		 * 
		 * @return std::string Formatted string of all the member objects
		 */
        std::string to_string() {
            return "X: " + std::to_string(this->getX().Convert(inch)) + " Y:" + std::to_string(this->getY().Convert(inch)) + " T:" + std::to_string(this->angle.Convert(degree));
        }

        /**
         * @brief Get the Angle as an Angle object
         * 
         * @return Angle Current angle
         */
        Angle getAngle() {
            return this->angle;
        }

		/**
		 * @brief Set the Angle as an Angle object
		 * 
		 * @param angle New angle
		 */
        void setAngle(Angle angle) {
            this->angle = angle;
        }

		/**
		 * @brief Set the current pose with the = operator with an object
		 * 
		 * @param pose The new pose
		 */
        void operator=(Pose2D pose) {
            this->setX(pose.getX());
            this->setY(pose.getY());
            this->setAngle(pose.getAngle());
        }

		/**
		 * @brief Set the current pose with the = operator with a pointer
		 * 
		 * @param pose The new pose
		 */
        void operator=(Pose2D* position) {
            this->setX(position->getX());
            this->setY(position->getY());
            this->setAngle(position->getAngle());
        }

		/**
		 * @brief Add function to add 2 poses together
		 * 
		 * @param b Other pose
		 * @return Pose2D Total of the poses
		 */
		Pose2D operator+(Pose2D b) {
			return Pose2D(this->getX() + b.getX(), this->getY() + b.getY(), this->getAngle() + b.getAngle());
		}

		/**
		 * @brief Add function to add 2 poses together
		 * 
		 * @param b Other point
		 * @return Pose2D Total of the poses
		 */
		Pose2D operator+(Point b) {
			return Pose2D(this->getX() + b.getX(), this->getY() + b.getY(), this->getAngle());
		}

        ~Pose2D();
    };

} // namespace Pronounce
