#pragma once

#include "api.h"
#include "utils/utils.hpp"
#include "utils/position.hpp"
#include "utils/pointUtil.hpp"
#include "utils/vector.hpp"
#include "position/odomWheel.hpp"
#include "odometry.hpp"

namespace Pronounce {

    class MecanumOdometry : Odometry {
    private:
        OdomWheel* wheel1;
        OdomWheel* wheel2;
        OdomWheel* wheel3;
        OdomWheel* wheel4;

        bool useImu;

        pros::Imu* imu;

        double wheelRadius;
        double xOffset;
        double yOffset;
    public:
        MecanumOdometry();
        MecanumOdometry(double wheelRadius, double xOffset, double yOffset);
        MecanumOdometry(OdomWheel* wheel1, OdomWheel* wheel2, OdomWheel* wheel3, OdomWheel* wheel4, pros::Imu* imu, double wheelRadius, double xOffset, double yOffset);

        void update();

        void reset(Position* position) {
            this->setPosition(position);
            this->setResetPosition(position);
            wheel1->reset();
            wheel2->reset();
            wheel3->reset();
            wheel4->reset();
            imu->set_rotation(position->getTheta());
        }

        /**
         * @brief Set the Wheel Radius object
         * 
         * @param wheelRadius 
         */
        void setWheelRadius(double wheelRadius) {
            this->wheelRadius = wheelRadius;
        }
        
        double getWheelRadius() {
            return wheelRadius;
        }

        /**
         * @brief Set the X Offset object
         * 
         * @param xOffset 
         */
        void setXOffset(double xOffset) {
            this->xOffset = xOffset;
        }

        /**
         * @brief Get the X Offset object
         * 
         * @return double 
         */
        double getXOffset() {
            return xOffset;
        }

        /**
         * @brief Set the Y Offset object
         * 
         * @param yOffset 
         */
        void setYOffset(double yOffset) {
            this->yOffset = yOffset;
        }

        /**
         * @brief Get the Y Offset object
         * 
         * @return double 
         */
        double getYOffset() {
            return yOffset;
        }
        
        OdomWheel* getWheel1() {
            return wheel1;
        }

        void setWheel1(OdomWheel *wheel1) {
            this->wheel1 = wheel1;
        }

        OdomWheel* getWheel2() {
            return wheel2;
        }

        void setWheel2(OdomWheel* wheel2) {
            this->wheel2 = wheel2;
        }

        OdomWheel* getWheel3() {
            return wheel3;
        }

        void setWheel3(OdomWheel* wheel3) {
            this->wheel3 = wheel3;
        }

        OdomWheel* getWheel4() {
            return wheel4;
        }

        void setWheel4(OdomWheel* wheel4) {
            this->wheel4 = wheel4;
        }
        
        /**
         * @brief Destroy the Mecanum Odometry object
         * 
         */
        ~MecanumOdometry();
    };
} // namespace Pronounce
