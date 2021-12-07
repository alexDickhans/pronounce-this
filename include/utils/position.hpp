#pragma once

#include "pointUtil.hpp"
#include <string>

namespace Pronounce {
    /**
     * A class to keep track of position on the field
     */
    class Position : public Point {
    private:
        double theta;
    public:
        Position();
        Position(double theta);
        Position(double x, double y);
        Position(double x, double y, double theta);

        std::string to_string() {
            return "X: " + std::to_string(this->getX()) + " Y:" + std::to_string(this->getY()) + " T:" + std::to_string(this->theta);
        }

        /**
         * Get the Y position
         *
         * @return Y
         */
        double getTheta() {
            return this->theta;
        }

        /**
         * Get the Y position
         *
         * @param Y Y position
         */
        void setTheta(double theta) {
            this->theta = theta;
        }

        void operator=(Position position) {
            this->setX(position.getX());
            this->setY(position.getY());
            this->setTheta(position.getTheta());
        }

        void operator=(Position* position) {
            this->setX(position->getX());
            this->setY(position->getY());
            this->setTheta(position->getTheta());
        }

        ~Position();
    };

} // namespace Pronounce
