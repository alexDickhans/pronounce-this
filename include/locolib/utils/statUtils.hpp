#pragma once

#include <cmath>

namespace Loco {
	class StatUtils {
	public:
		static double normal_pdf(const double & x) {
			return (1.0/(double)pow(2 * M_PI, 0.5)) * exp(-0.5 * x * x);
		}
	};
}