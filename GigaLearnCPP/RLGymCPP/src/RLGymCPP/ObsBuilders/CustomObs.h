#pragma once
#include "ObsBuilder.h"


namespace RLGC {
    // NOTE: This not based off of AdvancedObs in Python RLGym, and is specific to GigaLearn
    class CustomObs : public ObsBuilder {
    public:

        int maxPlayers;

        constexpr static float
            POS_COEF = 1 / 5000.f,
            VEL_COEF = 1 / 2300.f,
            ANG_VEL_COEF = 1 / 3.f;

        CustomObs(int maxPlayers = 3) : maxPlayers(maxPlayers) {}

        virtual void AddPlayerToObs(FList& obs, const Player& player, bool inv, const PhysState& ball);

        virtual FList BuildObs(const Player& player, const GameState& state) override;

    };
}
