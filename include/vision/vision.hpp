#pragma once

#include "api.h"
#include "utils/position.hpp"
#include "utils/pointCloud.hpp"
#include "utils/utils.hpp"

#define numItems 10 

namespace Pronounce
{
    class Vision : public pros::Vision {
    private:
        double xOffset = 0, yOffset = 0, zOffset = 0;
        double pitch = 0, yaw = 0;

        double xFov = 1.06, yFov = 0.72;
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
} // namespace PronounceTiP
