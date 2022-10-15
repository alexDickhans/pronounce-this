#pragma once

#include "continuousOdometry.hpp"
#include "position/odomWheel.hpp"
#include "utils/vector.hpp"
#include "utils/utils.hpp"
#include "utils/pose2d.hpp"
#include "odometry/orientation/orientation.hpp"

namespace Pronounce {

	/**
	 * @brief Odometry with 3 odometry wheels with 2 parallel and 1 perpendicular to those, no support for rotation in the setup yet
	 * 
	 */
    class ThreeWheelOdom : public ContinuousOdometry {
    private:
		/**
		 * @brief Odometry wheels
		 * 
		 */
        std::shared_ptr<OdomWheel> leftWheel, rightWheel, backWheel;

		/**
		 * @brief Offset from the center of rotation
		 * 
		 */
        QLength leftOffset, rightOffset, backOffset;

		/**
		 * @brief The inertial measurement unit that can be used for orientation
		 * 
		 */
		std::shared_ptr<Orientation> externalOrientation;

		/**
		 * @brief If true it will use the imu for orientation calculations instead of the parallel wheels
		 * 
		 */
		bool useExternalOrientation = false;

		Angle orientationOffset{90.0};

    public:
		/**
		 * @brief Construct a new Three Wheel Odom object with all values set to zero and no pointers set
		 * 
		 */
        ThreeWheelOdom();

		/**
		 * @brief Construct a new Three Wheel Odom object with all values set to zero and only odom wheels set
		 * 
		 * @param leftWheel The left odom wheel
		 * @param rightWheel The right odom wheel
		 * @param backWheel The back odom wheel
		 */
        ThreeWheelOdom(std::shared_ptr<OdomWheel> leftWheel, std::shared_ptr<OdomWheel> rightWheel, std::shared_ptr<OdomWheel> backWheel);

		/**
		 * @brief Construct a new Three Wheel Odom object with all values set to zero and odom and imus set
		 * 
		 * @param leftWheel The left odom wheel
		 * @param rightWheel The right odom wheel
		 * @param backWheel The back odom wheel
		 * @param imu The inertial measurement system for the robot
		 */
        ThreeWheelOdom(std::shared_ptr<OdomWheel> leftWheel, std::shared_ptr<OdomWheel> rightWheel, std::shared_ptr<OdomWheel> backWheel, std::shared_ptr<Orientation> externalOrientation);

		/**
		 * @brief Update all the values using the new data
		 * 
		 */
        void update();

		/**
		 * @brief Reset the pose and all the encoders
		 * 
		 * @param position The new position
		 */
        void reset(Pose2D position) {
            this->setPose(position);
            this->setResetPose(position);
            this->leftWheel->reset();
            this->rightWheel->reset();
            this->backWheel->reset();
        }

		/**
		 * @brief Get the Left Offset distance	
		 * 
		 * @return QLength The left offset distance
		 */
        QLength getLeftOffset() {
            return leftOffset;
        }

		/**
		 * @brief Set the Left Offset distance
		 * 
		 * @param leftOffset New left offset distance
		 */
        void setLeftOffset(QLength leftOffset) {
            this->leftOffset = leftOffset;
        }

		/**
		 * @brief Get the Right Offset distance
		 * 
		 * @return QLength The right offset distance
		 */
        QLength getRightOffset() {
            return rightOffset;
        }

		/**
		 * @brief Set the Right Offset distance
		 * 
		 * @param rightOffset New right offset distance
		 */
        void setRightOffset(QLength rightOffset) {
            this->rightOffset = rightOffset;
        }

		/**
		 * @brief Get the Back Offset distance
		 * 
		 * @return QLength The back offset distance
		 */
        QLength getBackOffset() {
            return backOffset;
        }

		/**
		 * @brief Set the Back Offset distance
		 * 
		 * @param backOffset New back offset distance
		 */
        void setBackOffset(QLength backOffset) {
            this->backOffset = backOffset;
        }

		/**
		 * @brief Get the Left Wheel odom wheel
		 * 
		 * @return OdomWheel* Pointer to the left odom wheel
		 */
        std::shared_ptr<OdomWheel> getLeftWheel() {
            return leftWheel;
        }

		/**
		 * @brief Set the Left Wheel odom wheel
		 * 
		 * @param leftWheel Pointer to the new left odom wheel
		 */
        void setLeftWheel(std::shared_ptr<OdomWheel> leftWheel) {
            this->leftWheel = leftWheel;
        }

		/**
		 * @brief Get the Right Wheel odom wheel
		 * 
		 * @return OdomWheel* Pointer to the right odom wheel
		 */
        std::shared_ptr<OdomWheel> getRightWheel() {
            return rightWheel;
        }

		/**
		 * @brief Set the Right Wheel odom wheel
		 * 
		 * @param rightWheel Pointer to the new right odom wheel
		 */
        void setRightWheel(std::shared_ptr<OdomWheel> rightWheel) {
            this->rightWheel = rightWheel;
        }

		/**
		 * @brief Get the Back Wheel odom wheel
		 * 
		 * @return OdomWheel* Pointer to the right odom wheel
		 */
        std::shared_ptr<OdomWheel> getBackWheel() {
            return backWheel;
        }

		/**
		 * @brief Set the Back Wheel odom wheel
		 * 
		 * @param backWheel Pointer to the new back odom wheel
		 */
        void setBackWheel(std::shared_ptr<OdomWheel> backWheel) {
            this->backWheel = backWheel;
        }

		bool getUseExternalOrientation() {
			return this->useExternalOrientation;
		}

		void setUseExternalOrientation(bool useExternalOrientation) {
			this->useExternalOrientation = useExternalOrientation;
		}

        ~ThreeWheelOdom();
    };
} // namespace Pronounce

