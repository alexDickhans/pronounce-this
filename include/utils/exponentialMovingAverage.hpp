#pragma once

namespace Pronounce
{
	class ExponentialMovingAverage {
	private:
		double lastEMA = 0.0;

		int loops;
		double smoothing;

	public:
		ExponentialMovingAverage(int loops, double smoothing) {
			this->loops = loops;
			this->smoothing = smoothing;
		}

		double update(double value) {
			lastEMA = (value * (smoothing/loops+1)) + 
						(lastEMA * (1-(smoothing/loops+1)));
			return lastEMA;
		}

		~ExponentialMovingAverage() {}
	};	
} // namespace Pronounce
