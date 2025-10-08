#include "CustomObs.h"
#include <RLGymCPP/Gamestates/StateUtil.h>

void RLGC::CustomObs::AddPlayerToObs(FList& obs, const Player& player, bool inv, const PhysState& ball) {
	auto phys = InvertPhys(player, inv);

	obs += phys.pos * POS_COEF;
	obs += phys.rotMat.forward;
	obs += phys.rotMat.up;
	obs += phys.vel * VEL_COEF;
	obs += phys.angVel * ANG_VEL_COEF;
	obs += phys.rotMat.Dot(phys.angVel) * ANG_VEL_COEF;

	// Local ball pos, vel, and angVel
	obs += phys.rotMat.Dot(ball.pos - phys.pos) * POS_COEF;
	obs += phys.rotMat.Dot(ball.vel - phys.vel) * VEL_COEF;

	obs += player.boost / 100;
	obs += player.isOnGround;
	obs += player.HasFlipOrJump();
	obs += player.isDemoed;
	obs += player.hasJumped; // Allows detecting flip resets
}

RLGC::FList RLGC::CustomObs::BuildObs(const Player& player, const GameState& state) {
	FList obs = {};

	bool inv = player.team == Team::ORANGE;

	auto ball = InvertPhys(state.ball, inv);
	auto phys = InvertPhys(player, inv);
	auto& pads = state.GetBoostPads(inv);
	auto& padTimers = state.GetBoostPadTimers(inv);

	obs += ball.pos * POS_COEF;
	obs += ball.vel * VEL_COEF;
	obs += ball.angVel * ANG_VEL_COEF;

	for (int i = 0; i < player.prevAction.ELEM_AMOUNT; i++)
		obs += player.prevAction[i];

	for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
		// A clever trick that blends the boost pads using their timers
		if (pads[i]) {
			obs += 1.f; // Pad is already available
		}
		else {
			obs += 1.f / (1.f + padTimers[i]); // Approaches 1 as the pad becomes available
		}
	}

	// Add self observation
	FList selfObs = {};
	AddPlayerToObs(selfObs, player, inv, ball);
	obs += selfObs;
	int playerObsSize = selfObs.size() + 9;

	std::vector<FList> teammates = {}, opponents = {};

	for (auto& otherPlayer : state.players) {
		if (otherPlayer.carId == player.carId)
			continue;

		auto otherPhys = InvertPhys(otherPlayer, inv);

		// Relative position and velocity (in local agent frame)
		Vec relPos = phys.rotMat.Dot(otherPhys.pos - phys.pos);
		Vec relVel = phys.rotMat.Dot(otherPhys.vel - phys.vel);

		Vec otherAngVel = phys.rotMat.Dot(otherPhys.angVel);

		FList playerObs;
		playerObs += relPos * POS_COEF;
		playerObs += relVel * VEL_COEF;

		playerObs += otherAngVel * ANG_VEL_COEF;

		AddPlayerToObs(playerObs, otherPlayer, inv, ball);
		(otherPlayer.team == player.team ? teammates : opponents).push_back(playerObs);
	}

	if (teammates.size() > maxPlayers - 1)
		RG_ERR_CLOSE("CustomObsV2: Too many teammates for Obs, maximum is " << (maxPlayers - 1));

	if (opponents.size() > maxPlayers)
		RG_ERR_CLOSE("CustomObsV2: Too many opponents for Obs, maximum is " << maxPlayers);

	for (int i = 0; i < 2; i++) {
		auto& playerList = i ? teammates : opponents;
		int targetCount = i ? maxPlayers - 1 : maxPlayers;

		while (playerList.size() < targetCount) {
			FList pad = FList(playerObsSize);
			playerList.push_back(pad);
		}
	}

	// Shuffle both lists
	std::shuffle(teammates.begin(), teammates.end(), ::Math::GetRandEngine());
	std::shuffle(opponents.begin(), opponents.end(), ::Math::GetRandEngine());

	// Add padded and shuffled teammates and opponents
	for (auto& teammate : teammates) obs += teammate;
	for (auto& opponent : opponents) obs += opponent;

	return obs;
}
