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

		PID* orientationPid;

		/**
		 * @brief Turn target for the drivetrain
		 * 
		 */
		Angle turnTarget{0.0};
	public:
		/**
		 * @brief Construct a new Omni Pure Pursuit object with most variables set
		 * 
		 * @param drivetrain Pointer to the drivetrain to drive
		 * @param odometry Pointer to the odometry object
		 * @param lookaheadDistance The target lookahead distance
		 */
		OmniPurePursuit(AbstractHolonomicDrivetrain* drivetrain, ContinuousOdometry* odometry, PurePursuitProfile currentProfile);
		
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

		/**
		 * @brief Get the Orientation Pid object
		 * 
		 * @return PID* The orientation pid object
		 */
		PID* getOrientationPid() {
			return orientationPid;
		}

		/**
		 * @brief Set the Orientation Pid object
		 * 
		 * @param orientationPid New Orientation pid object
		 */
		void setOrientationPid(PID* orientationPid) {
			this->orientationPid = orientationPid;
			this->orientationPid->setTurnPid(true);
		}

		/**
		 * @brief Get the Turn Target angle
		 * 
		 * @return Angle Get the turn angle
		 */
		Angle getTurnTarget() {
			return this->turnTarget;
		}

		/**
		 * @brief Set the Turn Target angle
		 * 
		 * @param turnTarget The turn angle
		 */
		void setTurnTarget(Angle turnTarget) {
			this->turnTarget = turnTarget;
		}

		~OmniPurePursuit();
	};
} // namespace Pronounce
