
#include "../../include/pathPlanner/utils/linearInterpolator.hpp"
#include <iostream>

int main() {
	auto interpolator = PathPlanner::LinearInterpolator();

	interpolator.add(0, 0);
	interpolator.add(1, 1);
	interpolator.add(2, 0);
	interpolator.add(3, -1);
	interpolator.add(4, 0);
	interpolator.add(5, 1);
	interpolator.add(6, 0);

//	std::cout << interpolator.to_string() << std::endl;

	for (int i = -100; i <= 600; i++) {

		std::cout << "Time:" << static_cast<double>(i)/100 << " Velocity:"
		          << interpolator.getIntegral(static_cast<double>(i)/100) << std::endl;
	}

	std::cout << "Done" << std::endl;
}