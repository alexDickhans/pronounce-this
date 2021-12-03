#include "purePursuitProfileManager.hpp"

namespace Pronounce {
    PurePursuitProfileManager::PurePursuitProfileManager() {
		this->setDefaultProfile(PurePursuitProfile());
	}

	PurePursuitProfileManager::PurePursuitProfileManager(PurePursuitProfile profiles ...) : PurePursuitProfileManager() {
		for (int i = 0; i < sizeof(profiles); i ++) {
			this->profiles.emplace_back(profiles);
		}
	}

	PurePursuitProfileManager::PurePursuitProfileManager(PurePursuitProfile defaultProfile) {
		this->defaultProfile = defaultProfile;
	}

	PurePursuitProfileManager::~PurePursuitProfileManager() {
	}
} // namespace Pronounce
