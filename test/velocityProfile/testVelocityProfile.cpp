
#define SIM

#include "utils/utils.cpp"
#include "../../include/units/units.hpp"
#include "../../include/velocityProfile/velocityProfile.hpp"
#include "../../include/velocityProfile/sinusoidalVelocityProfile.hpp"
#include <iostream>

int main() {
	Pronounce::SinusoidalVelocityProfile velocityProfile = Pronounce::SinusoidalVelocityProfile(50_in, 50_in/1_s, 200_in/second/second, 0.0);

	QTime frameTime = 10_ms;

	velocityProfile.calculate(10);

	std::cout << "Total time: " << velocityProfile.getDuration().Convert(second) << std::endl;

	for (QTime time = 0.0; time < velocityProfile.getDuration(); time += frameTime) {

		std::cout << "Time: " << time.Convert(second) << " Velocity: " << velocityProfile.getVelocityByTime(time).Convert(inch/second) << std::endl;
	}
}