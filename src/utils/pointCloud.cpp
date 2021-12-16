#include "pointCloud.hpp"

namespace Pronounce
{
    PointCloud::PointCloud(/* args */) {
    }
    
    void PointCloud::rotate(double angle) {
        for (int i = 0; i < points.size(); i++) {
            Vector vector = Vector(&points.at(i));
            vector.rotate(angle);
            points.at(i) = vector.getCartesian();
        }
    }

    void PointCloud::add(Point point) {
        for (int i = 0; i < points.size(); i++) {
            Point currentPoint = point;
            currentPoint.add(point);
            points.at(i) = currentPoint;
        }
    }

    PointCloud::~PointCloud() {
    }
} // namespace Pronounce
