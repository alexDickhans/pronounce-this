#pragma once

#include "pointUtil.hpp"
#include "vector.hpp"
#include <vector>

namespace Pronounce
{
    class PointCloud {
    private:
        std::vector<Point> points;
    public:
        PointCloud(/* args */);

        void rotate(double angle);

        void add(Point point);

        void reset() {
            points.clear();
        }

        Point getPoint(int index) {
            return points[index];
        }
        
        void setPoint(int index, Point point) {
            points[index] = point;
        }

        void addPoint(Point point) {
            points.push_back(point);
        }

        int getSize() {
            return points.size();
        }

        ~PointCloud();
    };   
} // namespace Pronounce
