#pragma once
#include "AdvancedObs.h"

namespace RLGC {
	// Version of AdvancedObs that supports a varying number of players (i.e. 1v1, 2v2, 3v3, etc.)
	// Maximum player count can be however high you want
	// Opponent and teammate slots are randomly shuffled to prevent slot bias
	class AdvancedObsPadded : public AdvancedObs {
	public:

		int maxPlayers;

		AdvancedObsPadded(
			int maxPlayers,
			float posCoef = 1 / 5000.f,
			float velCoef = 1 / 2300.f,
			float angVelCoef = 1 / 3.f
		) : AdvancedObs(posCoef, velCoef, angVelCoef), maxPlayers(maxPlayers) {

		}

		virtual FList BuildObs(const Player& player, const GameState& state);
	};
}
