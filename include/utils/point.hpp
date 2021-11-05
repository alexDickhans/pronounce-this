#pragma once

namespace Pronounce {
    class Point {
    private:
        double x, y;
    public:
        Point();
        Point(double x, double y);

        double getX() {
            return this->x;
        }

        void setX(double x) {
            this->x = x;
        }

        double getY() {
            return y;
        }

        void setY(double y) {
            this->y = y;
        }

        ~Point();
    };
} // namespace Pronounce
