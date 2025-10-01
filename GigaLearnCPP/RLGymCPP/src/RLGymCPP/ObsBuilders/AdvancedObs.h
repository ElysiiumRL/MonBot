#pragma once

#include "ObsBuilder.h"

namespace RLGC {
	// NOTE: This not based off of AdvancedObs in Python RLGym, and is specific to GigaLearn
	class AdvancedObs : public ObsBuilder {
	public:

		float posCoef, velCoef, angVelCoef;
		
		AdvancedObs(
			float posCoef = 1 / 5000.f,
			float velCoef = 1 / 2300.f,
			float angVelCoef = 1 / 3.f
		) : posCoef(posCoef), velCoef(velCoef), angVelCoef(angVelCoef) {

		}

		virtual void AddPlayerToObs(FList& obs, const Player& player, bool inv, const PhysState& ball);

		virtual FList BuildObs(const Player& player, const GameState& state) override;
	};
}