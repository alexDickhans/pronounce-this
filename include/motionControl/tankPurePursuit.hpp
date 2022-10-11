#pragma once

#include "purePursuit.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include <iostream>

namespace Pronounce {
	/**
	 * @brief Pure pursuit specifically for tank drive using curvature control
	 * 
	 * @authors Alex Dickhans(ad101-lab)
	 * 
	 */
	class TankPurePursuit : public PurePursuit {
	private:
		/**
		 * @brief Pointer to the drivetrain that pure pursuit will drive
		 * 
		 */
		AbstractTankDrivetrain* drivetrain;

		/**
		 * @brief Desired speed of the robot
		 * 
		 */
		QSpeed speed = 0.0;

	public:

		/**
		 * @brief Construct a new Tank Pure Pursuit object with a pointer to the drivetrain and a QLength distance
		 * 
		 * @param drivetrain Pointer to the drivetrain object that it will drive
		 * @param odometry Pointer to the odometry object that it will use for the lookahead point calculation
		 * @param lookaheadDistance QLength for the lookahead distance
		 */
		TankPurePursuit(std::string name, AbstractTankDrivetrain* drivetrain, ContinuousOdometry* odometry, PurePursuitProfile currentProfile, Path path);

		/**
		 * @brief Update the drivetrain based on the value from the pure pursuit parent object
		 * 
		 */
		void updateDrivetrain(PurePursuitPointData pointData);

		/**
		 * @brief Stop the drivetrain
		 * 
		 */
		void stop();

		/**
		 * @brief Get the Drivetrain Pointer
		 * 
		 * @return AbstractTankDrivetrain* pointer to the drivetrain object
		 */
		AbstractTankDrivetrain* getDrivetrain() {
			return drivetrain;
		}

		/**
		 * @brief Set the Drivetrain object via pointer
		 * 
		 * @param drivetrain Pointer to the drivetrain object
		 */
		void setDrivetrain(AbstractTankDrivetrain* drivetrain) {
			this->drivetrain = drivetrain;
		}

		/**
		 * @brief Get the desired speed of the drivetrain
		 * 
		 * @return QSpeed Current desired speed of the drivetrain
		 */
		QSpeed getSpeed() {
			return this->speed;
		}

		/**
		 * @brief Set the desired speed
		 * 
		 * @param speed Desired speed
		 */
		void setSpeed(QSpeed speed) {
			this->speed = speed;
		}

		/**
		 * @brief Determine if the drivetrain is within maxDistance of the end of the path
		 * 
		 * @param maxDistance Max distance of the path
		 * @return true It's done driving to the end of the path
		 * @return false Still more than max distance from the end of the path
		 */
		bool isDone(QLength maxDistance) {
			return maxDistance > this->getPath().distanceFromEnd(Point(this->getOdometry()->getPosition().getX(), this->getOdometry()->getPosition().getY()));
		}

		~TankPurePursuit();
	};
} // namespace Pronounce
