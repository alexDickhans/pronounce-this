#pragma once

#include "api.h"
#include "abstractHolonomicDrivetrain.hpp"
#include "utils/vector.hpp"
#include "odometry/threeWheelOdom.hpp"

namespace Pronounce {
	class MecanumDrivetrain : public AbstractHolonomicDrivetrain {
	private:
        pros::Motor* frontLeft;
        pros::Motor* frontRight;
        pros::Motor* backLeft;
        pros::Motor* backRight;

		Odometry* odometry;
	public:
        MecanumDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);
        MecanumDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, Odometry* odometry);

		void setDriveVectorVelocity(Vector vector);
		void setDriveVectorVelocity(Vector vector, double rotation);

        /**
         * Get frontLeft motor
         * 
         * @return frontLeft motor pointer
         */
        pros::Motor* getFrontLeft() {
            return this->frontLeft;
        }

        /**
         * Set frontLeft motor
         * 
         * @param frontLeft Motor pointer
         */
        void setFrontLeft(pros::Motor* frontLeft) {
            this->frontLeft = frontLeft;
        }
        
        /**
         * Get frontRight motor
         * 
         * @return frontRight motor pointer
         */
        pros::Motor* getFrontRight() {
            return this->frontRight;
        }

        /**
         * Set frontRight motor
         * 
         * @param frontRight Motor pointer
         */
        void setFrontRight(pros::Motor* frontRight) {
            this->frontRight = frontRight;
        }

        /**
         * Get backLeft motor
         * 
         * @return backLeft motor pointer
         */
        pros::Motor* getBackLeft() {
            return this->backLeft;
        }

        /**
         * Set backLeft motor
         * 
         * @param backLeft Motor pointer
         */
        void setBackLeft(pros::Motor* backLeft) {
            this->backLeft = backLeft;
        }

        /**
         * Get backRight motor
         * 
         * @return backRight motor pointer
         */
        pros::Motor* getBackRight() {
            return this->backRight;
        }

        /**
         * Set backRight motor
         * 
         * @param backRight Motor pointer
         */
        void setBackRight(pros::Motor* backRight) {
            this->backRight = backRight;
        }

		Odometry* getThreeWheelOdom() {
			return odometry;
		}

		void setThreeWheelOdom(Odometry* odometry) {
			this->odometry = odometry;
		}

		~MecanumDrivetrain();
	};
} // namespace Pronounce

