#pragma once

#include "api.h"
#include "omniDrivetrain.hpp"
#include "utils/vector.hpp"
#include "odometry/threeWheelOdom.hpp"

namespace Pronounce {
	class MecanumDrivetrain : public OmniDrivetrain {
	private:
		Odometry* odometry;
	public:
        MecanumDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);
        MecanumDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, Odometry* odometry);

		void setDriveVectorVelocity(Vector vector);
		void setDriveVectorVelocity(Vector vector, double rotation);

		Odometry* getThreeWheelOdom() {
			return odometry;
		}

		void setThreeWheelOdom(Odometry* odometry) {
			this->odometry = odometry;
		}

		~MecanumDrivetrain();
	};
} // namespace Pronounce

