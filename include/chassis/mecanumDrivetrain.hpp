#pragma once

#include "api.h"
#include "omniDrivetrain.hpp"
#include "utils/vector.hpp"
#include "odometry/threeWheelOdom.hpp"

namespace Pronounce {
	class MecanumDrivetrain : public OmniDrivetrain {
	private:
		ThreeWheelOdom* threeWheelOdom;
	public:
        MecanumDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);
        MecanumDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, ThreeWheelOdom* threeWheelOdom);

		void setDriveVectorVelocity(Vector vector);
		void setDriveVectorVelocity(Vector vector, double rotation);

		ThreeWheelOdom* getThreeWheelOdom() {
			return threeWheelOdom;
		}

		void setThreeWheelOdom(ThreeWheelOdom* threeWheelOdom) {
			this->threeWheelOdom = threeWheelOdom;
		}

		~MecanumDrivetrain();
	};
} // namespace Pronounce

