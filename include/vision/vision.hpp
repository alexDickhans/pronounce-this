#pragma once

#include "api.h"
#include "utils/position.hpp"
#include "utils/pointCloud.hpp"

#define numItems 10 

namespace Pronounce
{
    class Vision : public pros::Vision {
    private:
        double xOffset = 0, yOffset = 0, zOffset = 0;
        double pitch = 0, yaw = 0;

        double xFov = 61, yFov = 41;
        double xPixels = 640, yPixels = 400;

    public:
        Vision(std::uint8_t port, pros::vision_zero_e_t zero_point = pros::E_VISION_ZERO_CENTER);
        Vision(std::uint8_t port, double xOffset, double yOffset, double zOffset, pros::vision_zero_e_t zero_point = pros::E_VISION_ZERO_CENTER);

        PointCloud getLocalPointCloud(int signature, size_t maxSigs);

        double getXOffset() {
            return xOffset;
        }

        void setXOffset(double xOffset) {
            this->xOffset = xOffset;
        }

        double getYOffset() {
            return yOffset;
        }

        void setYOffset(double yOffset) {
            this->yOffset = yOffset;
        }

        double getZOffset() {
            return zOffset;
        }

        void setZOffset(double zOffset) {
            this->zOffset = zOffset;
        }

        double getPitch() {
            return pitch;
        }

        void setPitch(double pitch) {
            this->pitch = pitch;
        }

        double getYaw() {
            return yaw;
        }

        void setYaw(double yaw) {
            this->yaw = yaw;
        }

        ~Vision();
    };

    PointCloud Vision::getLocalPointCloud(int signature, size_t maxSigs) {
        int objectCount = this->get_object_count();

        pros::vision_object_s_t objects[maxSigs];
        this->read_by_sig(0, signature, maxSigs, objects);

        for (int i = 0; maxSigs < i; i++) {
            pros::vision_object_s_t object = objects[i];

            double objectXAngle = map(object.x_middle_coord, 0, xPixels, 0, xFov) + yaw;
            double objectYAngle = map(object.y_middle_coord, 0, yPixels, 0, yFov) + pitch;

            double localX = tan(objectXAngle) * yOffset;
        }
        

    }
} // namespace PronounceTiP
