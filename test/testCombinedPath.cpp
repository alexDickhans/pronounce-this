//
// Created by alex on 9/11/23.
//

#include <iostream>
#include "units/units.hpp"
#include "utils/path/combinedPath.hpp"

using namespace Pronounce;

int main() {
	std::cout << "Hello World" << std::endl;

	CombinedPath path = CombinedPath({{10_in, 15_deg/10_in}, {10_in, -15_deg/10_in}});

	for (int i = 0; i <= 100; i++) {
//		std::cout << "i: " << i << " distance: " << i/100.0 * path.getDistance().Convert(inch) << " angle: " << path.getAngleAtDistance(i/100.0 * path.getDistance()).Convert(degree) << std::endl;
		std::cout << "angle: " << path.getAngleAtT(i/100.0).Convert(degree) << std::endl;
		printf("hello World");
	}
}