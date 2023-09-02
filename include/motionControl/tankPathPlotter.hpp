//
// Created by alex on 8/27/23.
//

#ifndef PRONOUNCE_THIS_TANKPATHPLOTTER_HPP
#define PRONOUNCE_THIS_TANKPATHPLOTTER_HPP

#include "chassis/abstractTankDrivetrain.hpp"

namespace Pronounce {

    class TankPathPlotter {
    private:
		static pros::Mutex mutex_;
        static TankPathPlotter* instance;
        AbstractTankDrivetrain* drivetrain;

        explicit TankPathPlotter(AbstractTankDrivetrain* drivetrain) {
			this->drivetrain = drivetrain;
        }
    public:
        TankPathPlotter(TankPathPlotter const&) = delete;
        void operator=(TankPathPlotter const&) = delete;

        static TankPathPlotter* getInstance(AbstractTankDrivetrain* newDrivetrain = nullptr) {
			std::lock_guard<pros::Mutex> lock(mutex_);

			if (instance == nullptr) {
				assert(newDrivetrain != nullptr);
				instance = new TankPathPlotter(newDrivetrain);
			}

			return instance;
        }


    };



} // Pronounce

#endif //PRONOUNCE_THIS_TANKPATHPLOTTER_HPP
