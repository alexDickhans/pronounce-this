#pragma once

#include "continuousOdometry/continuousOdometry.hpp"

namespace Pronounce {
	class OdomFuser : public ContinuousOdometry {
	private:
		/* data */
	public:
		OdomFuser(/* args */);
		~OdomFuser();
	};
	
	OdomFuser::OdomFuser(/* args */)
	{
	}
	
	OdomFuser::~OdomFuser()
	{
	}
	
} // namespace Pronounce
