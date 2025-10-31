#include "CustomObs.h"
#include <RLGymCPP/Gamestates/StateUtil.h>
#include <RLGymCPP/CommonValues.h>
#include <algorithm>

using namespace RLGC;

FList CustomObs::BuildObs(const Player& player, const GameState& state) {
    FList obs = {};

    bool inv = player.team == Team::ORANGE;

    // États physiques inversés si nécessaire
    auto ball = InvertPhys(state.ball, inv);
    auto phys = InvertPhys(player, inv);
    const auto& pads = state.GetBoostPads(inv);
    const auto& padTimers = state.GetBoostPadTimers(inv);

    // ==================== BALLE (9) ====================
    // Position monde
    obs += ball.pos * POS_COEF;
    // Vélocité
    obs += ball.vel * VEL_COEF;
    // Vélocité angulaire
    obs += ball.angVel * ANG_VEL_COEF;

    // ==================== BALLE DANS REPÈRE JOUEUR (6) ====================
    // Position locale
    Vec localBallPos = phys.rotMat.Dot(ball.pos - phys.pos);
    obs += localBallPos * POS_COEF;
    // Vélocité locale
    Vec localBallVel = phys.rotMat.Dot(ball.vel - phys.vel);
    obs += localBallVel * VEL_COEF;

    // ==================== JOUEUR (18) ====================
    // Position monde
    obs += phys.pos * POS_COEF;
    // Orientation (forward + up)
    obs += phys.rotMat.forward;
    obs += phys.rotMat.up;
    // Vélocité linéaire
    obs += phys.vel * VEL_COEF;
    // Vélocité angulaire (world)
    obs += phys.angVel * ANG_VEL_COEF;
    // Vélocité angulaire (locale)
    Vec localAngVel = phys.rotMat.Dot(phys.angVel);
    obs += localAngVel * ANG_VEL_COEF;

    // ==================== ÉTAT JOUEUR (7) ====================
    // Boost
    obs += player.boost / 100.f;
    // Au sol
    obs += player.isOnGround ? 1.f : 0.f;
    // A un flip/jump disponible
    obs += player.HasFlipOrJump() ? 1.f : 0.f;
    // Demoed
    obs += player.isDemoed ? 1.f : 0.f;
    // A sauté
    obs += player.hasJumped ? 1.f : 0.f;
    // Supersonic
    obs += player.isSupersonic ? 1.f : 0.f;
    // Double jump utilisé
    obs += player.hasDoubleJumped ? 1.f : 0.f;

    // ==================== RELATIONS BALLE-JOUEUR (6) ====================
    Vec toBall = ball.pos - phys.pos;
    float distToBall = toBall.Length();
    // Distance à la balle
    obs += distToBall * POS_COEF;
    // Alignement forward vers balle
    obs += phys.rotMat.forward.Dot(toBall.Normalized());
    // Vitesse relative vers balle
    float relSpeed = std::max(0.f, phys.vel.Dot(toBall.Normalized()));
    obs += relSpeed / CommonValues::CAR_MAX_SPEED;
    // Différence hauteur
    obs += (ball.pos.z - phys.pos.z) * POS_COEF;
    // Kickoff detection
    bool isKickoff = (std::abs(ball.pos.x) < 20.f && std::abs(ball.pos.y) < 20.f && ball.vel.Length() < 50.f);
    obs += isKickoff ? 1.f : 0.f;
    // Verticalité chassis
    obs += phys.rotMat.up.z;

    // ==================== BUTS (8) ====================
    Vec oppGoal(0, CommonValues::BACK_WALL_Y, 0);
    Vec ownGoal(0, -CommonValues::BACK_WALL_Y, 0);

    // Distance joueur -> but adverse
    Vec toOppGoal = oppGoal - phys.pos;
    obs += toOppGoal.Length() * POS_COEF;
    // Direction locale vers but adverse
    obs += phys.rotMat.Dot(toOppGoal.Normalized());

    // Distance balle -> but adverse  
    Vec ballToOppGoal = oppGoal - ball.pos;
    obs += ballToOppGoal.Length() * POS_COEF;

    // Distance joueur -> propre but
    Vec toOwnGoal = ownGoal - phys.pos;
    obs += toOwnGoal.Length() * POS_COEF;

    // ==================== PROXIMITÉ TERRAIN (4) ====================
    float dx = CommonValues::SIDE_WALL_X - std::abs(phys.pos.x);
    float dy = CommonValues::BACK_WALL_Y - std::abs(phys.pos.y);
    float dz = CommonValues::CEILING_Z - phys.pos.z;
    obs += dx * POS_COEF;
    obs += dy * POS_COEF;
    obs += dz * POS_COEF;
    obs += std::min({dx, dy, dz}) * POS_COEF;

    // ==================== BOOST LE PLUS PROCHE (2) ====================
    int nearestIdx = -1;
    float bestDist2 = 1e20f;
    for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
        int mapIdx = inv ? (CommonValues::BOOST_LOCATIONS_AMOUNT - i - 1) : i;
        const Vec& padPos = CommonValues::BOOST_LOCATIONS[mapIdx];
        Vec d = padPos - phys.pos;
        float d2 = d.Dot(d);
        if (d2 < bestDist2) {
            bestDist2 = d2;
            nearestIdx = i;
        }
    }
    if (nearestIdx >= 0) {
        obs += std::sqrt(bestDist2) * POS_COEF;
        obs += pads[nearestIdx] ? 1.f : (1.f / (1.f + padTimers[nearestIdx]));
    } else {
        obs += 0.f;
        obs += 0.f;
    }

    // ==================== TOUS LES BOOST PADS (34) ====================
    for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
        obs += pads[i] ? 1.f : (1.f / (1.f + padTimers[i]));
    }

    // ==================== DERNIÈRE ACTION (8) ====================
    for (int i = 0; i < player.prevAction.ELEM_AMOUNT; i++) {
        obs += player.prevAction[i];
    }

    // ==================== BALL PREDICTION SIMPLE (21) ====================
    // Prédiction linéaire basique reproductible en Python
    // 3 horizons : 0.2s, 0.5s, 1.0s
    constexpr float GRAVITY = 650.f;
    constexpr float BALL_RADIUS = 93.15f;
    std::vector<float> horizons = {0.2f, 0.5f, 1.0f};

    for (float t : horizons) {
        // Prédiction linéaire simple
        Vec predPos = ball.pos + ball.vel * t;
        Vec predVel = ball.vel + Vec(0, 0, -GRAVITY) * t;

        // Collision sol simple
        if (predPos.z < BALL_RADIUS) {
            float timeToGround = (ball.pos.z - BALL_RADIUS) / std::max(1.f, -ball.vel.z);
            if (timeToGround > 0 && timeToGround < t) {
                // Rebond au sol (perte d'énergie 60%)
                predPos.z = BALL_RADIUS;
                predVel.z = std::abs(ball.vel.z - GRAVITY * timeToGround) * 0.6f;
            } else {
                predPos.z = BALL_RADIUS;
                predVel.z = 0.f;
            }
        }

        // Position prédite dans mon repère
        Vec localPredPos = phys.rotMat.Dot(predPos - phys.pos);
        obs += localPredPos * POS_COEF;

        // Vélocité prédite dans mon repère
        Vec localPredVel = phys.rotMat.Dot(predVel - phys.vel);
        obs += localPredVel * VEL_COEF;

        // Distance à cette position future
        float distToPred = (predPos - phys.pos).Length();
        obs += distToPred * POS_COEF;
    }

    // ==================== ADVERSAIRE (38) ====================
    // Trouver l'adversaire en 1v1
    const Player* opp = nullptr;
    for (auto& p : state.players) {
        if (p.carId != player.carId && p.team != player.team) {
            opp = &p;
            break;
        }
    }

    if (opp) {
        auto oppPhys = InvertPhys(*opp, inv);

        // Position relative (locale)
        Vec relPos = phys.rotMat.Dot(oppPhys.pos - phys.pos);
        obs += relPos * POS_COEF;
        
        // Vélocité relative (locale)
        Vec relVel = phys.rotMat.Dot(oppPhys.vel - phys.vel);
        obs += relVel * VEL_COEF;
        
        // Vélocité angulaire adversaire (locale à moi)
        Vec oppAngVelLocal = phys.rotMat.Dot(oppPhys.angVel);
        obs += oppAngVelLocal * ANG_VEL_COEF;

        // Orientation adversaire (dans mon repère)
        Vec oppFwdLocal = phys.rotMat.Dot(oppPhys.rotMat.forward);
        obs += oppFwdLocal;
        Vec oppUpLocal = phys.rotMat.Dot(oppPhys.rotMat.up);
        obs += oppUpLocal;

        // État adversaire
        obs += opp->boost / 100.f;
        obs += opp->isOnGround ? 1.f : 0.f;
        obs += opp->HasFlipOrJump() ? 1.f : 0.f;
        obs += opp->isDemoed ? 1.f : 0.f;
        obs += opp->hasJumped ? 1.f : 0.f;
        obs += opp->isSupersonic ? 1.f : 0.f;
        obs += opp->hasDoubleJumped ? 1.f : 0.f;

        // Relations adversaire-balle
        Vec oppToBall = ball.pos - oppPhys.pos;
        float oppDistToBall = oppToBall.Length();
        obs += oppDistToBall * POS_COEF;
        // Alignement adversaire vers balle
        obs += oppPhys.rotMat.forward.Dot(oppToBall.Normalized());
        
        // Distance adversaire -> but adverse
        Vec oppToOppGoal = oppGoal - oppPhys.pos;
        obs += oppToOppGoal.Length() * POS_COEF;
        
        // Qui est plus proche de la balle
        obs += (distToBall < oppDistToBall) ? 1.f : 0.f;
    } else {
        // Padding si pas d'adversaire (38 zéros)
        for (int i = 0; i < 38; i++) {
            obs += 0.f;
        }
    }

    return obs;
}
