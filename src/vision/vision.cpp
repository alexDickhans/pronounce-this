#include "vision.hpp"

namespace Pronounce {
    PointCloud Vision::getLocalPointCloud(int signature, size_t maxSigs) {

        // Read the vision sensor signatures
        pros::vision_object_s_t objects[maxSigs];
        this->read_by_sig(0, signature, maxSigs, objects);

        // Create a point cloud
        PointCloud cloud;

        for (int i = 0; maxSigs < i; i++) {
            // Get the object
            pros::vision_object_s_t object = objects[i];

            // Get the object's position
            double objectXAngle = map(object.x_middle_coord, 0, xPixels, 0, xFov) + yaw;
            double objectYAngle = map(object.y_middle_coord, 0, yPixels, 0, yFov) + pitch;

            // Create vectors with the object's position
            Vector xVector(1, objectXAngle);
            Vector yVector(1, objectYAngle);

            // Scale the vectors properly
            xVector.scale(zOffset/xVector.getCartesian().getY());
            yVector.scale(zOffset/yVector.getCartesian().getY());

            // Add the vectors together
            Point currentPoint = Point(xVector.getCartesian().getX() + xOffset, yVector.getCartesian().getX() + yOffset);

            // Add the point to the cloud
            cloud.addPoint(currentPoint);
        }
        
        // Return the point cloud
        return cloud;
    }
} // namespace Pronounce
