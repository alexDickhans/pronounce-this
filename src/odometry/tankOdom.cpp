#include "tankOdom.hpp"

namespace Pronounce
{
	TankOdom::TankOdom(OdomWheel* odomWheel, pros::Imu* imu) {
		this->odomWheel = odomWheel;
		this->imu = imu;

		this->setPosition(new Position());
	}
	
	void TankOdom::reset() {
		this->imu->reset();
	}

	void TankOdom::update() {
		odomWheel->update();

		double average = odomWheel->getChange();
		double angle = toRadians(imu->get_heading());

		double magnitude = average;

		double x2 = magnitude * cos(angle);
		double y2 = magnitude * sin(angle);

		if (std::isnan(x2) || std::isnan(y2)) {
			return;
		}

		this->getPosition()->setX(this->getPosition()->getX() + x2);
		this->getPosition()->setY(this->getPosition()->getY() + y2);
		this->getPosition()->setTheta(angle);
	}

	TankOdom::~TankOdom() {
	}
} // namespace Pronounce
