#pragma once
#include "ObsBuilder.h"

namespace RLGC {
    // CustomObs ultra optimisé 1v1 sans ball prediction
    // Observation compacte et rapide focalisée sur l'essentiel
    class CustomObs : public ObsBuilder {
    public:
        // Coefficients de normalisation
        constexpr static float
            POS_COEF = 1.f / 5000.f,
            VEL_COEF = 1.f / 2300.f,
            ANG_VEL_COEF = 1.f / 3.f;

        CustomObs() = default;
        virtual ~CustomObs() = default;

        virtual FList BuildObs(const Player& player, const GameState& state) override;
    };
}
