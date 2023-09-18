//
// Created by alex on 9/11/23.
//

#include <iostream>
#include "units/units.hpp"
#include "utils/path/combinedPath.hpp"

using namespace Pronounce;

int main() {
	std::cout << "Hello World" << std::endl;

	CombinedPath path = CombinedPath({{30_in, 0.0}, {10_in, 0.0}});

	for (int i = 0; i <= 10; i++) {
		std::cout << "i: " << i << " distance: " << i/10.0 * path.getDistance().Convert(inch) << " angle: " << path.getSegmentAtDistance(i/10.0 * path.getDistance()).curvature.Convert(degree/inch) << std::endl;
//		std::cout << "angle: " << path.getAngleAtT(i/100.0).Convert(degree) << std::endl;
	}
}