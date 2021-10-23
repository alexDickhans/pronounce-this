#include "tankOdom.hpp"

namespace Pronounce
{
	TankOdom::TankOdom(MotorOdom* leftPivot, MotorOdom* rightPivot, pros::Imu* imu) {
		this->leftPivot = leftPivot;
		this->rightPivot = rightPivot;
		this->imu = imu;

		this->position = new Position();
	}
	
	void TankOdom::reset() {
		this->imu->reset();
	}

	void TankOdom::update() {
		leftPivot->update();
		rightPivot->update();

		double average = ((leftPivot->getChange() + rightPivot->getChange()) * tuningFactor) / 2;
		double angle = toRadians(imu->get_heading());

		double x1 = 0;
		double y1 = average;

		double x2 = (x1 * cos(angle)) + (y1 * sin(angle));
		double y2 = (x1 * sin(angle)) + (y1 * cos(angle));

		if (std::isnan(x2) || std::isnan(y2)) {
			return;
		}

		this->position->setX(this->position->getX() + x2);
		this->position->setY(this->position->getY() + y2);
		this->position->setTheta(angle);
	}

	TankOdom::~TankOdom() {
	}
} // namespace Pronounce
