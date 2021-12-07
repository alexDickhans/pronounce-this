#pragma once

#include "purePursuitProfile.hpp"
#include <vector>

namespace Pronounce {
	class PurePursuitProfileManager {
	private:
		std::vector<PurePursuitProfile> profiles;
		PurePursuitProfile defaultProfile;
	public:
		PurePursuitProfileManager();
		PurePursuitProfileManager(PurePursuitProfile defaultProfile);
		PurePursuitProfileManager(PurePursuitProfile profiles ...);

		/**
		 * @brief Add a profile to the vector
		 *
		 * @param profile The profile to add
		 * @return int The index of the profile
		 */
		int add(PurePursuitProfile profile) {
			this->profiles.emplace_back(profile);
			return profiles.size() - 1;

		}

		/**
		 * @brief Get the Profile object at index
		 *
		 * @param index The index of the profile
		 * @return PurePursuitProfile The profile at index, or the default one if the array is out of bound
		 */
		PurePursuitProfile getProfile(int index) {
			if (index >= profiles.size()) {
				return defaultProfile;
			}
			return this->profiles.at(index);
		}

		PurePursuitProfile getDefaultProfile() {
			return defaultProfile;
		}

		void setDefaultProfile(PurePursuitProfile defaultProfile) {
			this->defaultProfile = defaultProfile;
			printf("Default profile lookahead distance: %f\n", defaultProfile.getLookaheadDistance());
			printf("Member lookahead distance: %f\n", this->getDefaultProfile().getLookaheadDistance());
		} 

		~PurePursuitProfileManager();
	};
} // namespace Pronounce
