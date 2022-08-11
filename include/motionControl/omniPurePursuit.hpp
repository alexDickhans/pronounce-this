#pragma once

#include "chassis/abstractHolonomicDrivetrain.hpp"
#include "purePursuit.hpp"
#include "utils/vector.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"

namespace Pronounce {

	/**
	 * @brief A pure pursuit class for omnidirectional drivetrain
	 * 
	 * @authors Alex Dickhans (ad101-lab)
	 * 
	 */
	class OmniPurePursuit : public PurePursuit {
	private:

		/**
		 * @brief The pointer to the drivetrain
		 * 
		 */
		AbstractHolonomicDrivetrain* drivetrain;

		Angle turnTarget{0.0};
	public:

		/**
		 * @brief Construct a new Omni Pure Pursuit object with default values and no drivetrain object
		 * 
		 */
		OmniPurePursuit();

		/**
		 * @brief Construct a new Omni Pure Pursuit object with drivetrain pointer ser
		 * 
		 * @param drivetrain Pointer to the drivetrain that you want to drive
		 */
		OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain);

		/**
		 * @brief Construct a new Omni Pure Pursuit object with a drivetrain pointer and lookahead value 
		 * 
		 * @param drivetrain Pointer to the drivetrain to drive
		 * @param lookaheadDistance The target lookahead distance
		 */
		OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain, QLength lookaheadDistance);

		/**
		 * @brief Construct a new Omni Pure Pursuit object with most variables set
		 * 
		 * @param drivetrain Pointer to the drivetrain to drive
		 * @param odometry Pointer to the odometry object
		 * @param lookaheadDistance The target lookahead distance
		 */
		OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain, ContinuousOdometry* odometry, QLength lookaheadDistance);
		
		/**
		 * @brief Update the drivetrain object to more the drivetrain, not called by the user
		 * 
		 * @param pointData The pointdata given from the parent class
		 */
		void updateDrivetrain(PurePursuitPointData pointData);

		/**
		 * @brief Stop all the motors
		 * 
		 */
		void stop();

		/**
		 * @brief Get the Drivetrain pointer
		 * 
		 * @return AbstractHolonomicDrivetrain* Pointer to the driven drivetrain
		 */
		AbstractHolonomicDrivetrain* getDrivetrain() {
			return drivetrain;
		}

		/**
		 * @brief Set the Drivetrain pointer
		 * 
		 * @param drivetrain The new pointer to the drivetrain object
		 */
		void setDrivetrain(AbstractHolonomicDrivetrain* drivetrain) {
			this->drivetrain = drivetrain;
		}

		~OmniPurePursuit();
	};
} // namespace Pronounce
