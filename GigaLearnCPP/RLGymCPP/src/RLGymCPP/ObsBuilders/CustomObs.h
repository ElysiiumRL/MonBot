#pragma once
#include "ObsBuilder.h"
#include <RLGymCPP/Gamestates/StateUtil.h>

namespace RLGC {
    /**
     * ULTRA CUSTOM OBS - Configuration maximale avec vision prédictive complète
     *
     * Fichiers consultés:
     * - CustomObs.h/cpp (base)
     * - AdvancedObs.h/cpp (référence)
     * - Car.h (états physiques détaillés)
     * - Ball.h (prédictions)
     * - BallHitInfo.h (touches)
     * - Arena.h (méthodes prédictives)
     * - RLConst.h (constantes physiques)
     */
    class CustomObs : public ObsBuilder {
    public:
        int maxPlayers;
        bool enableBallPrediction;
        bool enableTacticalFeatures;
        int ballPredictionSteps;

        // Coefficients de normalisation optimaux (issus de RLConst.h)
        constexpr static float
            POS_COEF = 1.f / 5000.f,           // Max arena extent
            VEL_COEF = 1.f / 6000.f,           // BALL_MAX_SPEED
            CAR_VEL_COEF = 1.f / 2300.f,       // CAR_MAX_SPEED
            ANG_VEL_COEF = 1.f / 6.f,          // BALL_MAX_ANG_SPEED
            CAR_ANG_VEL_COEF = 1.f / 5.5f,     // CAR_MAX_ANG_VEL
            TIME_COEF = 1.f / 5.f,             // Normalisation temps (5s max)
            BOOST_COEF = 1.f / 100.f,          // Boost max
            HEIGHT_COEF = 1.f / 2044.f;        // Arena height

        CustomObs(
            int maxPlayers = 3,
            bool enableBallPrediction = true,
            bool enableTacticalFeatures = true,
            int ballPredictionSteps = 20
        ) : maxPlayers(maxPlayers),
            enableBallPrediction(enableBallPrediction),
            enableTacticalFeatures(enableTacticalFeatures),
            ballPredictionSteps(ballPredictionSteps) {
        }

        // Ajoute les features de la balle avec prédictions
        virtual void AddBallFeatures(FList& obs, const PhysState& ball, const GameState& state);

        // Ajoute les features du joueur (ultra-détaillé)
        virtual void AddPlayerToObs(FList& obs, const Player& player, bool inv, const PhysState& ball, const PhysState& agentPhys);

        // Ajoute les features tactiques avancées
        virtual void AddTacticalFeatures(FList& obs, const Player& agent, const GameState& state, bool inv);

        // Calcule les prédictions de balle
        virtual void AddBallPredictions(FList& obs, const GameState& state, bool inv);

        // Construit l'observation complète
        virtual FList BuildObs(const Player& player, const GameState& state) override;

    private:
        // Helper: Distance entre deux points
        inline float Distance(const Vec& a, const Vec& b) const {
            return (a - b).Length();
        }

        // Helper: Angle entre deux vecteurs
        inline float AngleBetween(const Vec& a, const Vec& b) const {
            float dot = a.Dot(b);
            float mag = a.Length() * b.Length();
            if (mag < 0.0001f) return 0.f;
            return acosf(RS_CLAMP(dot / mag, -1.f, 1.f));
        }
    };
}
