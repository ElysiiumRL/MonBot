#pragma once
#include <RLGymCPP/ObsBuilders/ObsBuilder.h>
#include <RLGymCPP/Gamestates/GameState.h>
#include <RLGymCPP/Gamestates/Player.h>
#include <RLGymCPP/Gamestates/StateUtil.h>
#include <cmath>

namespace RLGC {
    class AdvancedObs : public ObsBuilder {
    public:
        int teamSize;
        float POS_STD, ANG_STD;
        bool expanding;

        AdvancedObs(int teamSize = 3, bool expanding = false);

        // This signature matches the GigaLearn ObsBuilder interface
        virtual FList BuildObs(const Player& player, const GameState& state) override;

    private:
        void AddDummy(FList& obs);

        // Helper function signature corrected to match the implementation
        PhysState AddPlayerToObs(FList& obs, const Player& player, const PhysState& ball, bool inv);
    };
}
