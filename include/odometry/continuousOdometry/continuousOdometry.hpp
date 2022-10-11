#pragma once

#include "utils/pose2d.hpp"
#include "utils/vector.hpp"

// TODO: Clean code

namespace Pronounce {
	/**
	 * @brief Continuous odometry where it will always have a value no matter the other conditions
	 * 
	 * @authors Alex Dickhans (alexDickhans)
	 * 
	 */
    class ContinuousOdometry {
    private:
		/**
		 * @brief The updated pose of the robot
		 * 
		 */
        Pose2D pose;

		/**
		 * @brief The pose on the last reset
		 * 
		 */
        Pose2D resetPose;

		/**
		 * @brief The current velocity and direction of the velocity in the form of a vector
		 * 
		 */
		Vector currentVelocity;
    public:

		/**
		 * @brief Construct a new Continuous Odometry object with all values set to zero
		 * 
		 */
        ContinuousOdometry();

		/**
		 * @brief Construct a new Continuous Odometry object with reset position set
		 * 
		 * @param pose 
		 */
        ContinuousOdometry(Pose2D* pose);

		/**
		 * @brief Get the Position object in a pointer
		 * 
		 * @return Pose2D* Pointer to the current position
		 */
        Pose2D getPosition() {
            return this->pose;
        }

		/**
		 * @brief Get the current x position
		 * 
		 * @return QLength The x distance from the bottom left corner of the field
		 */
		QLength getX() {
			return this->pose.getX();
		}

		/**
		 * @brief Get the current y position 
		 * 
		 * @return QLength The y distance from the bottom of the field
		 */
		QLength getY() {
			return this->pose.getY();
		}

		/**
		 * @brief Get the Angle
		 * 
		 * @return Angle Angle from the driver station
		 */
		Angle getAngle() {
			return this->pose.getAngle();
		}

		/**
		 * @brief Get the Pose object with the position and angle
		 * 
		 * @return Pose2D* Pose2D relative to the bottom left corner of the field
		 */
		Pose2D getPose() {
			return pose;
		}

		/**
		 * @brief Set the Pose object with the position and angle - don't use
		 * 
		 * @warning Don't use
		 * 
		 * @param pose The new pose
		 */
        void setPose(Pose2D pose) {
            this->pose = pose;
        }

		/**
		 * @brief Get the last reset position
		 * 
		 * @return Pose2D* The last reset pose
		 */
        Pose2D getResetPose() {
            return this->resetPose;
        }

		/**
		 * @brief Set the current reset pose - don't use
		 * 
		 * @param resetPose 
		 */
        void setResetPose(Pose2D resetPose) {
            this->resetPose = resetPose;
        }
		
		/**
		 * @brief Get the Current Velocity vector
		 * 
		 * @return Vector The current velocity vector
		 */
		Vector getCurrentVelocity() {
			return this->currentVelocity;
		}

		/**
		 * @brief Set the Current Velocity object, should only be used for child classes
		 * 
		 * @param velocity 
		 */
		void setCurrentVelocity(Vector velocity) {
			this->currentVelocity = velocity;
		}

		/**
		 * @brief An update function to call every frame at as quick of an update cycle as possible, implmented by child classes
		 * 
		 */
        virtual void update() {};

		/**
		 * @brief Set the position of the robot
		 * 
		 * @param pose The current pose of the robot
		 */
        virtual void reset(Pose2D pose) {
            this->pose.operator=(pose);
            this->resetPose.operator=(pose);
			this->currentVelocity.operator=(Vector());
        }

		/**
		 * @brief Like the other reset function, but sets to the bottom left corner
		 * 
		 */
        void reset() {
            this->reset(new Pose2D());
        }

        ~ContinuousOdometry();
    };    
} // namespace Pronounce
