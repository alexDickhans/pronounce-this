#pragma once

#include "api.h"
#include "utils/position.hpp"

#define numItems 10 

namespace PronounceTiP
{
    class Vision : public pros::Vision {
    private:
        double xOffset = 0, yOffset = 0, zOffset = 0;
        double xAngle = 0, yAngle = 0, zAngle = 0;

        double xFov = 61, yFov = 41;
        double xPixels = 640, yPixels = 400;

        pros::vision_color_code_t rings;
        pros::vision_color_code_t goals;

        pros::vision_object_s_t objectArr[numItems];
        Pronounce::Position ringsAngle[numItems];
        Pronounce::Position goalsAngle[numItems];
    public:
        Vision(std::uint8_t port, pros::vision_zero_e_t zero_point = pros::E_VISION_ZERO_CENTER);
        Vision(std::uint8_t port, double xOffset, double yOffset, double zOffset, pros::vision_zero_e_t zero_point = pros::E_VISION_ZERO_CENTER);

        void updateAngles();
        void getAnglesBySig(int sig_id, Pronounce::Position* positions);

        ~Vision();
    };
} // namespace PronounceTiP
