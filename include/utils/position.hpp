#pragma once

namespace Pronounce
{
    /**
     * A class to keep track of position on the field
     */
    class Position
    {
    private:
        double X, Y, theta;
    public:
        Position();
        Position(double theta);
        Position(double X, double Y);
        Position(double X, double Y, double theta);

        /**
         * Get the X position
         *
         * @return X
         */
        double getX() {
            return this->X;
        }

        /**
         * Get the X position
         *
         * @param X X position
         */
        void setX(double X) {
            this->X = X;
        }

        /**
         * Get the Y position
         *
         * @return Y
         */
        double getY() {
            return this->Y;
        }

        /**
         * Get the Y position
         *
         * @param Y Y position
         */
        void setY(double Y) {
            this->Y = Y;
        }

        /**
         * Get the Y position
         *
         * @return Y
         */
        double getTheta() {
            return this->Y;
        }

        /**
         * Get the Y position
         *
         * @param Y Y position
         */
        void setTheta(double Y) {
            this->Y = Y;
        }

        ~Position();
    };

} // namespace Pronounce
