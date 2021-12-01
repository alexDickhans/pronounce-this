#include "tippingVision.hpp"
#include "utils/utils.hpp"
#include "okapi/api.hpp"


namespace PronounceTiP
{

    Vision::Vision(std::uint8_t port, pros::vision_zero_e_t zero_point) : pros::Vision(port, zero_point) {
        this->print_signature(this->get_signature(1));
    }

    Vision::Vision(std::uint8_t port, double xOffset, double yOffset, double zOffset, pros::vision_zero_e_t zero_point) : pros::Vision(port, zero_point) {
    }

    Vision::~Vision() {
    }

    void Vision::updateAngles() {

        this->getAnglesBySig(1, ringsAngle);
        this->getAnglesBySig(2, goalsAngle);
    }

    void Vision::getAnglesBySig(int sig_id, Pronounce::Position* positions) {

        if (this->get_object_count() < 1) {
            int index = this->read_by_sig(0, sig_id, numItems, objectArr);

            for (int i = 0; i < index; i++) {
                double x = objectArr[i].x_middle_coord; // Pronounce::map(objectArr[i].x_middle_coord, -320, 320, -30.5, 30.5);
                double y = objectArr[i].y_middle_coord; // Pronounce::map(objectArr[i].y_middle_coord, -200, 200, -20.5, 20.5);
                positions[i].setX(x);
                positions[i].setY(y);
            }
            // visionLogger.get()->debug<std::string>(Pronounce::string_format("Object Count: %d, X: %d, Y: %d", this->get_object_count(), positions[0].getX(), positions[0].getY()));

        }

        // visionLogger.get()->debug<std::string>(Pronounce::string_format("Object Count: %d", this->get_object_count()));

    }


} // namespace PronounceTiP
