#pragma once

#include "api.h"
#include "drivetrain.hpp"
#include "abstractTankDrivetrain.hpp"
#include "utils/motorGroup.hpp"

namespace Pronounce {
	class TankDrivetrain : public AbstractTankDrivetrain, public Drivetrain {
	private:

	public:
	
		~TankDrivetrain();
	};
} // namespace Pronounce




