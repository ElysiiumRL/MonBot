#pragma once
#include "ObsBuilder.h"
#include <vector>
#include <unordered_map>

// Forward declarations dans le bon namespace
namespace RocketSim {
    class Arena;
    struct BallPredTracker;
}

namespace RLGC {
    // Ultra Custom Obs pour 1v1 avec ball prediction intégrée + features riches.
    class CustomObs : public ObsBuilder {
    public:
        // 1v1: un opposant max, pas de teammate.
        int maxPlayers;

        // Normalisations
        constexpr static float
            POS_COEF = 1 / 5000.f,
            VEL_COEF = 1 / 2300.f,
            ANG_VEL_COEF = 1 / 3.f;

        // Horizons de prédiction (secondes)
        std::vector<float> predHorizons;

        // Cache de trackers par arène
        std::unordered_map<RocketSim::Arena*, RocketSim::BallPredTracker*> predTrackers;

        CustomObs(
            int maxPlayers = 1,
            std::vector<float> predHorizons = { 0.10f, 0.20f, 0.40f, 0.80f, 1.20f }
        ) : maxPlayers(maxPlayers), predHorizons(std::move(predHorizons)) {
        }

        virtual ~CustomObs();

        // Features détaillées d’un joueur (self ou autre)
        virtual void AddPlayerToObs(FList& obs, const Player& player, bool inv, const PhysState& ball);

        // Observation complète pour 'player'
        virtual FList BuildObs(const Player& player, const GameState& state) override;

    private:
        void EnsurePredTracker(RocketSim::Arena* arena);
        size_t ComputePredTicks(RocketSim::Arena* arena) const;
    };
}
