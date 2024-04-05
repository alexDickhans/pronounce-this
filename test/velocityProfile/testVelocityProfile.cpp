
#define SIM

#include "../../src/utils/utils.cpp"
#include "../../include/units/units.hpp"
#include "../../include/velocityProfile/velocityProfile.hpp"
#include "../../src/velocityProfile/velocityProfile.cpp"
#include "../../include/velocityProfile/trapezoidalVelocityProfile.hpp"
#include "../../src/velocityProfile/trapezoidalVelocityProfile.cpp"
// #include "../../include/velocityProfile/sCurveVelocityProfile.hpp"
#include <iostream>

int main() {
	Pronounce::TrapezoidalVelocityProfile velocityProfile =
			Pronounce::TrapezoidalVelocityProfile(-3_in,
			                                      {70_in / second,
			                                       100_in / second /
			                                       second, 0.0},
			                                      0_in / second,
			                                      -30_in / second);

	QTime frameTime = 10_ms;

	velocityProfile.calculate();

	// std::cout << "Total time: " << velocityProfile.getDuration().Convert(second) << std::endl;

	for (QTime time = 0.0; time <= velocityProfile.getDuration(); time += frameTime) {

		std::cout << "Time:" << time.Convert(second) << " Velocity:"
		          << velocityProfile.getVelocityByTime(time).Convert(inch / second) << std::endl;
//		std::cout << "Time:" << time.Convert(second) << " Distance:" << velocityProfile.getDistanceByTime(time).Convert(inch) << std::endl;
	}

	std::cout << "Done" << std::endl;
}