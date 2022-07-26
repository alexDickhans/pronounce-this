#pragma once

#include "pointUtil.hpp"
#include <string>
#include "units/units.hpp"

namespace Pronounce {
    /**
     * A class to keep track of position on the field
     */
    class Pose2D : public Point {
    private:
        Angle angle;
    public:
        Pose2D();
        Pose2D(double angle);
        Pose2D(double x, double y);
        Pose2D(double x, double y, double angle);

        std::string to_string() {
            return "X: " + std::to_string(this->getX().Convert(inch)) + " Y:" + std::to_string(this->getY().Convert(inch)) + " T:" + std::to_string(this->angle.Convert(degree));
        }

        /**
         * Get the Y position
         *
         * @return Y
         */
        Angle getAngle() {
            return this->angle;
        }

        /**
         * Get the Y position
         *
         * @param Y Y position
         */
        void setAngle(Angle angle) {
            this->angle = angle;
        }

        void operator=(Pose2D pose) {
            this->setX(pose.getX());
            this->setY(pose.getY());
            this->setAngle(pose.getAngle());
        }

        void operator=(Pose2D* position) {
            this->setX(position->getX());
            this->setY(position->getY());
            this->setAngle(position->getAngle());
        }

        ~Pose2D();
    };

} // namespace Pronounce
