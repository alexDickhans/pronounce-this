#pragma once

// TODO: Add comments

namespace Pronounce {
	/**
	 * @brief Feedback controller parent class
	 * 
	 * @authors Alex Dickhans (alexDickhans) 
	 */
	class FeedbackController {
	private:
	protected:
		/**
		 * @brief Target position of the feedback controller
		 * 
		 */
		double target = 0.0;

		/**
		 * @brief The current position of the feedback controller
		 * 
		 */
		double position = 0.0;

		/**
		 * @brief The output power
		 * 
		 */
		double power{0.0};

		/**
		 * @brief The maximum power
		 * 
		 */
		double maxPower = 1.0;
	public:
		/**
		 * @brief Default constructor
		 * 
		 */
		FeedbackController(/* args */) = default;
		
		/**
		 * @brief Update the feedback controller and 
		 * 
		 * @param input The sensor value from this loop
		 * @return double The output of the loop
		 */
		virtual double update(double input) { return 0; }

		/**
		 * @brief Reset all the values
		 * 
		 */
		virtual void reset() {}

		/**
		 * @brief Set the Target value
		 * 
		 * @param target New target value
		 */
		void setTarget(double target) {
			this->target = target;
		}

		/**
		 * @brief Get the Target value
		 * 
		 * @return double Target value
		 */
		double getTarget() {
			return target;
		}

		/**
		 * @brief Set the Position/input value
		 * 
		 * @param target The new position value
		 */
		void setPosition(double target) {
			this->target = target;
		}

		/**
		 * @brief Get the Position value
		 * 
		 * @return double The position value
		 */
		double getPosition() {
			return position;
		}

		/**
		 * @brief Get the Power value
		 * 
		 * @return double The power from the last loop
		 */
		double getPower() {
			return power;
		}

		/**
		 * @brief Set the Max Power value
		 * 
		 * @param maxPower New max power
		 */
		void setMaxPower(double maxPower) {
			this->maxPower = maxPower;
		}

		/**
		 * @brief Get the Max Power object
		 * 
		 * @return double The current max power
		 */
		double getMaxPower() {
			return maxPower;
		}

		~FeedbackController() {}
	};
} // namespace Pronounce
