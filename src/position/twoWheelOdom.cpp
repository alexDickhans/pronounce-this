#include "twoWheelOdom.hpp"

namespace Pronounce
{

	TwoWheelOdom::TwoWheelOdom(OdomWheel* horizontal, OdomWheel* vertical, pros::Imu* imu, double offset) {
		this->vertical = vertical;
		this->horizontal = horizontal;
		this->imu = imu;
		this->offset = offset;
	}

	void TwoWheelOdom::update() {
		this->horizontal->update();
		this->vertical->update();

		double angle = this->imu->get_rotation() - this->offset;
		double x1 = this->horizontal->getChange();
		double y1 = this->vertical->getChange();

		double x2 = (x1 * sin(angle - 45)) - (y1 * cos(angle - 45));
		double y2 = (x1 * sin(angle - 45)) + (y1 * cos(angle - 45));

		this->position->setX(this->position->getX() + x2);
		this->position->setY(this->position->getY() + y2);
	}

	TwoWheelOdom::~TwoWheelOdom() {
	}
} // namespace Pronounce
