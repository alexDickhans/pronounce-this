#pragma once

namespace Pronounce {
	/**
	 * @brief Abstract class to be used in both the pure pursuit simulator and on the robot.
	 * 
	 */
	class AbstractDrivetrain {
	private:
		
	public:
		AbstractDrivetrain() {}

		/**
		 * @brief Get the current speed of the drivetrain
		 * 
		 * @return QSpeed 
		 */
		virtual QSpeed getSpeed() { return 0.0; }

		~AbstractDrivetrain() {}
	};
		
} // namespace Pronounce
