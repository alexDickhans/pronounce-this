#include "mecanumDrivetrain.hpp"

namespace Pronounce {
    MecanumDrivetrain::MecanumDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : OmniDrivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
	}

    MecanumDrivetrain::MecanumDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, ThreeWheelOdom* threeWheelOdom) : OmniDrivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
        this->threeWheelOdom = threeWheelOdom;
	}

	void MecanumDrivetrain::setDriveVectorVelocity(Vector vector) {
		this->getFrontLeft()->move_velocity(vector.getCartesian().getY() + vector.getCartesian().getX());
		this->getFrontRight()->move_velocity(vector.getCartesian().getY() - vector.getCartesian().getX());
		this->getBackLeft()->move_velocity(vector.getCartesian().getY() - vector.getCartesian().getX());
		this->getBackRight()->move_velocity(vector.getCartesian().getY() + vector.getCartesian().getX());
	}

	void MecanumDrivetrain::setDriveVectorVelocity(Vector vector, double rotation) {
		this->getFrontLeft()->move_velocity(vector.getCartesian().getY() + vector.getCartesian().getX() + rotation);
		this->getFrontRight()->move_velocity(vector.getCartesian().getY() - vector.getCartesian().getX() - rotation);
		this->getBackLeft()->move_velocity(vector.getCartesian().getY() - vector.getCartesian().getX() + rotation);
		this->getBackRight()->move_velocity(vector.getCartesian().getY() + vector.getCartesian().getX() - rotation);
	}
	
	MecanumDrivetrain::~MecanumDrivetrain() {
	}
} // namespace Pronounce