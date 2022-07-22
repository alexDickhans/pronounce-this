#pragma once

#include "pointUtil.hpp"
#include <string>

namespace Pronounce {
    /**
     * A class to keep track of position on the field
     */
    class Pose2D : public Point {
    private:
        double angle;
    public:
        Pose2D();
        Pose2D(double angle);
        Pose2D(double x, double y);
        Pose2D(double x, double y, double theta);

        std::string to_string() {
            return "X: " + std::to_string(this->getX()) + " Y:" + std::to_string(this->getY()) + " T:" + std::to_string(this->angle);
        }

        /**
         * Get the Y position
         *
         * @return Y
         */
        double getAngle() {
            return this->angle;
        }

        /**
         * Get the Y position
         *
         * @param Y Y position
         */
        void setAngle(double angle) {
            this->angle = angle;
        }

        void operator=(Pose2D position) {
            this->setX(position.getX());
            this->setY(position.getY());
            this->setAngle(position.getAngle());
        }

        void operator=(Pose2D* position) {
            this->setX(position->getX());
            this->setY(position->getY());
            this->setAngle(position->getAngle());
        }

        ~Pose2D();
    };

} // namespace Pronounce
