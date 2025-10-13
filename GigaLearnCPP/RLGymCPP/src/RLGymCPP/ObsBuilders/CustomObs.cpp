#include "CustomObs.h"
#include <RLGymCPP/Gamestates/StateUtil.h>

using namespace RLGC;

void CustomObs::AddBallFeatures(FList& obs, const PhysState& ball, const GameState& state) {
    // Position et vélocité de base
    obs += ball.pos * POS_COEF;
    obs += ball.vel * VEL_COEF;
    obs += ball.angVel * ANG_VEL_COEF;

    // Distances aux buts
    Vec blueGoal(0, -5120, 0);
    Vec orangeGoal(0, 5120, 0);
    obs += Distance(ball.pos, blueGoal) * POS_COEF;
    obs += Distance(ball.pos, orangeGoal) * POS_COEF;

    // Hauteur normalisée et vitesse scalaire
    obs += ball.pos.z * HEIGHT_COEF;
    float speed = ball.vel.Length();
    obs += speed * VEL_COEF;

    // Direction du mouvement (normalisée)
    Vec velDir = speed > 1.f ? ball.vel / speed : Vec(0, 0, 0);
    obs += velDir;

    // Prédictions de scoring (si Arena disponible)
    // Note: Ces méthodes nécessitent l'accès à l'Arena
    // obs += probGoalBlue; // Arena::IsBallProbablyGoingIn
    // obs += probGoalOrange;
    // obs += timeToGround;
}

void CustomObs::AddPlayerToObs(
    FList& obs,
    const Player& player,
    bool inv,
    const PhysState& ball,
    const PhysState& agentPhys
) {
    auto phys = InvertPhys(player, inv);

    // === PHYSIQUE DE BASE (18) ===
    obs += phys.pos * POS_COEF;                        // 3
    obs += phys.rotMat.forward;                        // 3
    obs += phys.rotMat.up;                             // 3
    obs += phys.rotMat.right;                          // 3 (NOUVEAU)
    obs += phys.vel * CAR_VEL_COEF;                    // 3
    obs += phys.angVel * CAR_ANG_VEL_COEF;             // 3

    // === VÉLOCITÉ ANGULAIRE LOCALE (3) ===
    obs += phys.rotMat.Dot(phys.angVel) * CAR_ANG_VEL_COEF;

    // === POSITION/VÉL BALLE RELATIVE (7) ===
    Vec relBallPos = phys.rotMat.Dot(ball.pos - phys.pos);
    Vec relBallVel = phys.rotMat.Dot(ball.vel - phys.vel);
    obs += relBallPos * POS_COEF;                      // 3
    obs += relBallVel * VEL_COEF;                      // 3
    obs += Distance(phys.pos, ball.pos) * POS_COEF;    // 1 - Distance balle

    // === ÉTATS BOOLÉENS (11) ===
    obs += player.boost * BOOST_COEF;                  // 1
    obs += (float)player.isOnGround;                   // 1
    obs += (float)player.HasFlipOrJump();              // 1
    obs += (float)player.isDemoed;                     // 1
    obs += (float)player.hasJumped;                    // 1
    obs += (float)player.hasDoubleJumped;              // 1
    obs += (float)player.isFlipping;                   // 1
    obs += (float)player.isSupersonic;                 // 1

    // Roues en contact (4)
    for (int i = 0; i < 4; i++)
        obs += (float)player.wheelsWithContact[i];

    // === TIMERS NORMALISÉS (4) ===
    obs += player.flipTime * (1.f / 0.65f);            // FLIP_TORQUE_TIME
    obs += player.jumpTime * (1.f / 0.2f);             // JUMP_MAX_TIME
    obs += player.airTime * TIME_COEF;                 // 1
    obs += player.supersonicTime * TIME_COEF;          // 1
    obs += player.handbrakeVal;                        // 1 (déjà [0,1])

    // === BALLHITINFO (8) ===
    obs += (float)player.ballHitInfo.isValid;                              // 1
    obs += player.ballHitInfo.relativePosOnBall * (1.f / 200.f);          // 3
    obs += player.ballHitInfo.extraHitVel * CAR_VEL_COEF;                 // 3

    // Ticks depuis dernière touche (normalisé sur 2 secondes à 120Hz)
    float ticksSinceHit = player.ballHitInfo.isValid ?
        (float)(player.updateCounter - player.ballHitInfo.tickCountWhenHit) : 240.f;
    obs += ticksSinceHit / 240.f;                                          // 1
}

void CustomObs::AddTacticalFeatures(
    FList& obs,
    const Player& agent,
    const GameState& state,
    bool inv
) {
    auto agentPhys = InvertPhys(agent, inv);
    auto ball = InvertPhys(state.ball, inv);

    // Distance au plus proche adversaire
    float closestOppDist = 99999.f;
    for (auto& player : state.players) {
        if (player.team != agent.team && !player.isDemoed) {
            auto oppPhys = InvertPhys(player, inv);
            float dist = Distance(agentPhys.pos, oppPhys.pos);
            closestOppDist = RS_MIN(closestOppDist, dist);
        }
    }
    obs += closestOppDist * POS_COEF;

    // Angle défensif (angle entre agent, balle et son but)
    Vec ownGoal = Vec(0, -5120, 0);
    Vec toBall = ball.pos - agentPhys.pos;
    Vec toGoal = ownGoal - agentPhys.pos;
    float defensiveAngle = AngleBetween(toBall, toGoal);
    obs += defensiveAngle / M_PI;

    // Avantage positionnel (distance balle-agent vs distance balle-adversaire le plus proche)
    float agentToBall = Distance(agentPhys.pos, ball.pos);
    float oppToBall = 99999.f;
    for (auto& player : state.players) {
        if (player.team != agent.team && !player.isDemoed) {
            auto oppPhys = InvertPhys(player, inv);
            oppToBall = RS_MIN(oppToBall, Distance(oppPhys.pos, ball.pos));
        }
    }
    float advantage = (oppToBall - agentToBall) / 10000.f;
    obs += RS_CLAMP(advantage, -1.f, 1.f);

    // Temps d'interception estimé (simplifié)
    float speed = agentPhys.vel.Length();
    float timeToIntercept = (speed > 100.f) ? (agentToBall / speed) : 5.f;
    obs += RS_MIN(timeToIntercept, 5.f) * TIME_COEF;
}

void CustomObs::AddBallPredictions(FList& obs, const GameState& state, bool inv) {
    // Note: Nécessite BallPredTracker dans GameState
    // Pour l'instant, on simule avec des prédictions simples

    auto ball = InvertPhys(state.ball, inv);
    float dt = 1.f / 120.f; // Tick time à 120Hz

    // Prédictions à 0.25s, 0.5s, 1s (pas de 5, 10, 20 ticks)
    std::vector<int> predSteps = { 5, 10, 20 };

    for (int step : predSteps) {
        float time = step * dt;

        // Prédiction linéaire simple (à améliorer avec BallPredTracker)
        Vec predPos = ball.pos + ball.vel * time;
        predPos.z += 0.5f * -650.f * time * time; // Gravité

        obs += predPos * POS_COEF; // 3 floats par prédiction
    }
}

FList CustomObs::BuildObs(const Player& player, const GameState& state) {
    FList obs = {};

    bool inv = player.team == Team::ORANGE;
    auto ball = InvertPhys(state.ball, inv);
    auto agentPhys = InvertPhys(player, inv);
    auto& pads = state.GetBoostPads(inv);
    auto& padTimers = state.GetBoostPadTimers(inv);

    // === 1. FEATURES BALLE (20) ===
    AddBallFeatures(obs, ball, state);

    // === 2. ACTIONS PRÉCÉDENTES (8) ===
    for (int i = 0; i < player.prevAction.ELEM_AMOUNT; i++)
        obs += player.prevAction[i];

    // === 3. BOOST PADS (34) ===
    for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
        if (pads[i]) {
            obs += 1.f;
        }
        else {
            obs += 1.f / (1.f + padTimers[i]); // Approche 1 quand disponible
        }
    }

    // === 4. FEATURES AGENT (58) ===
    FList selfObs = {};
    AddPlayerToObs(selfObs, player, inv, ball, agentPhys);
    obs += selfObs;
    int playerObsSize = selfObs.size() + 9; // +9 pour position/vel relative

    // === 5. FEATURES TACTIQUES (4) ===
    if (enableTacticalFeatures) {
        AddTacticalFeatures(obs, player, state, inv);
    }

    // === 6. PRÉDICTIONS BALLE (9) ===
    if (enableBallPrediction) {
        AddBallPredictions(obs, state, inv);
    }

    // === 7. COÉQUIPIERS ET ADVERSAIRES ===
    std::vector<FList> teammates = {}, opponents = {};

    for (auto& otherPlayer : state.players) {
        if (otherPlayer.carId == player.carId)
            continue;

        auto otherPhys = InvertPhys(otherPlayer, inv);

        // Position/vélocité relative dans le frame de l'agent
        Vec relPos = agentPhys.rotMat.Dot(otherPhys.pos - agentPhys.pos);
        Vec relVel = agentPhys.rotMat.Dot(otherPhys.vel - agentPhys.vel);
        Vec otherAngVel = agentPhys.rotMat.Dot(otherPhys.angVel);

        FList playerObs;
        playerObs += relPos * POS_COEF;              // 3
        playerObs += relVel * CAR_VEL_COEF;          // 3
        playerObs += otherAngVel * CAR_ANG_VEL_COEF; // 3

        AddPlayerToObs(playerObs, otherPlayer, inv, ball, agentPhys);
        (otherPlayer.team == player.team ? teammates : opponents).push_back(playerObs);
    }

    // Padding pour taille fixe
    if (teammates.size() > maxPlayers - 1)
        RG_ERR_CLOSE("UltraCustomObs: Trop de coéquipiers, max = " << (maxPlayers - 1));
    if (opponents.size() > maxPlayers)
        RG_ERR_CLOSE("UltraCustomObs: Trop d'adversaires, max = " << maxPlayers);

    while (teammates.size() < maxPlayers - 1)
        teammates.push_back(FList(playerObsSize, 0.f));
    while (opponents.size() < maxPlayers)
        opponents.push_back(FList(playerObsSize, 0.f));

    // Shuffle pour invariance de position
    std::shuffle(teammates.begin(), teammates.end(), ::Math::GetRandEngine());
    std::shuffle(opponents.begin(), opponents.end(), ::Math::GetRandEngine());

    // Ajout final
    for (auto& teammate : teammates) obs += teammate;
    for (auto& opponent : opponents) obs += opponent;

    return obs;
}
