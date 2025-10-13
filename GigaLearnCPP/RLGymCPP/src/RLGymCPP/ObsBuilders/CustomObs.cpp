#include "CustomObs.h"
#include <RLGymCPP/Gamestates/StateUtil.h>
#include <RLGymCPP/CommonValues.h>
#include <../RLGymCPP/RocketSim/src/Sim/BallPredTracker/BallPredTracker.h>

#include <algorithm>
#include <cmath>

using namespace RLGC;

static inline float Clamp01(float v) { return v < 0 ? 0.f : (v > 1.f ? 1.f : v); }

static void AddGoalFeatures(FList& obs, const PhysState& self, const PhysState& ball) {
    const Vec oppGoal(0, CommonValues::BACK_WALL_Y, 0);
    const Vec ownGoal(0, -CommonValues::BACK_WALL_Y, 0);

    // Car -> Opp Goal
    Vec carToOpp = oppGoal - self.pos;
    float carToOppDist = carToOpp.Length();
    obs += carToOppDist * CustomObs::POS_COEF;
    obs += self.rotMat.Dot(carToOpp).Normalized();

    // Ball -> Opp Goal (monde)
    Vec ballToOpp = oppGoal - ball.pos;
    float ballToOppDist = ballToOpp.Length();
    obs += ballToOppDist * CustomObs::POS_COEF;
    obs += ballToOpp.Normalized();

    // Car -> Own Goal
    Vec carToOwn = ownGoal - self.pos;
    float carToOwnDist = carToOwn.Length();
    obs += carToOwnDist * CustomObs::POS_COEF;
    obs += self.rotMat.Dot(carToOwn).Normalized();
}

static void AddFieldProximity(FList& obs, const PhysState& self) {
    // Distances aux murs/plafond (normalisées)
    float dx = CommonValues::SIDE_WALL_X - std::abs(self.pos.x);
    float dy = CommonValues::BACK_WALL_Y - std::abs(self.pos.y);
    float dz = CommonValues::CEILING_Z - self.pos.z;

    obs += dx * CustomObs::POS_COEF;
    obs += dy * CustomObs::POS_COEF;
    obs += dz * CustomObs::POS_COEF;

    // Min distance mur/plafond
    float dmin = std::min({ dx, dy, dz });
    obs += dmin * CustomObs::POS_COEF;

    // Tilt du chassis (1=verticalité parfaite, proche 0 = sur mur/plafond)
    obs += self.rotMat.up.z;
}

static void AddCarBallExtras(FList& obs, const PhysState& self, const PhysState& ball, const Player& player) {
    Vec toBall = ball.pos - self.pos;
    float distBall = toBall.Length();
    obs += distBall * CustomObs::POS_COEF;

    Vec dirBallLocal = self.rotMat.Dot(toBall).Normalized();
    obs += dirBallLocal;

    float fwdAlign = self.rotMat.forward.Dot(toBall.Normalized());
    obs += fwdAlign;

    float relSpeedToBall = std::max(0.f, self.vel.Dot(toBall.Normalized()));
    obs += relSpeedToBall / CommonValues::CAR_MAX_SPEED;

    obs += (ball.pos.z - self.pos.z) * CustomObs::POS_COEF;

    bool isKickoff = (std::abs(ball.pos.x) < 20.f) &&
        (std::abs(ball.pos.y) < 20.f) &&
        (ball.vel.Length() < 50.f);
    obs += isKickoff ? 1.f : 0.f;

    obs += player.isSupersonic ? 1.f : 0.f;
}

static void AddNearestBoostFeatures(
    FList& obs,
    const PhysState& self,
    const std::vector<bool>& pads,
    const std::vector<float>& padTimers,
    bool inv
) {
    int nearestIdx = -1;
    float bestDist2 = 1e20f;

    for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
        int mapIdx = inv ? (CommonValues::BOOST_LOCATIONS_AMOUNT - i - 1) : i;
        const Vec& padPos = CommonValues::BOOST_LOCATIONS[mapIdx];

        Vec d = padPos - self.pos;
        float d2 = d.Dot(d);
        if (d2 < bestDist2) {
            bestDist2 = d2;
            nearestIdx = i;
        }
    }

    if (nearestIdx >= 0) {
        float dist = std::sqrt(bestDist2);
        obs += dist * CustomObs::POS_COEF;

        if (pads[nearestIdx]) {
            obs += 1.f;
        }
        else {
            float t = padTimers[nearestIdx];
            obs += 1.f / (1.f + t);
        }
    }
    else {
        obs += 0.f;
        obs += 0.f;
    }
}

static void AddBallLineShots(FList& obs, const PhysState& ball) {
    // Heuristique simple: la balle va-t-elle vers un but sur trajectoire linéaire ?
    auto on_target = [](const PhysState& b, float goalY) {
        float vy = b.vel.y;
        if (std::abs(vy) < 1e-3f) return 0.f;
        float t = (goalY - b.pos.y) / vy;
        if (t <= 0) return 0.f;
        Vec hit = b.pos + b.vel * t;
        bool betweenPosts = std::abs(hit.x) <= CommonValues::GOAL_WIDTH_FROM_CENTER && hit.z > 0 && hit.z <= CommonValues::GOAL_HEIGHT;
        return betweenPosts ? 1.f : 0.f;
        };

    float shotOpp = on_target(ball, CommonValues::BACK_WALL_Y);
    float shotOwn = on_target(ball, -CommonValues::BACK_WALL_Y);

    obs += shotOpp;
    obs += shotOwn;
}

// ===================== CustomObs impl =====================

CustomObs::~CustomObs() {
    for (auto& kv : predTrackers) delete kv.second;
    predTrackers.clear();
}

void CustomObs::EnsurePredTracker(RocketSim::Arena* arena) {
    if (!arena) return;
    auto it = predTrackers.find(arena);
    size_t neededTicks = ComputePredTicks(arena);

    if (it == predTrackers.end()) {
        predTrackers[arena] = new RocketSim::BallPredTracker(arena, neededTicks);
    }
    else {
        auto* tracker = it->second;
        if (tracker->numPredTicks < neededTicks) {
            delete tracker;
            predTrackers[arena] = new RocketSim::BallPredTracker(arena, neededTicks);
        }
    }
}

size_t CustomObs::ComputePredTicks(RocketSim::Arena* arena) const {
    if (!arena || predHorizons.empty()) return 0;
    float maxH = 0.f;
    for (float t : predHorizons) maxH = std::max(maxH, t);
    return static_cast<size_t>(std::ceil(maxH / arena->tickTime)) + 3; // marge
}

void CustomObs::AddPlayerToObs(FList& obs, const Player& player, bool inv, const PhysState& ball) {
    auto phys = InvertPhys(player, inv);

    // Etat absolu (monde inversé)
    obs += phys.pos * POS_COEF;
    obs += phys.rotMat.forward;
    obs += phys.rotMat.up;
    obs += phys.vel * VEL_COEF;
    obs += phys.angVel * ANG_VEL_COEF;
    obs += phys.rotMat.Dot(phys.angVel) * ANG_VEL_COEF;

    // Balle locale à ce joueur
    obs += phys.rotMat.Dot(ball.pos - phys.pos) * POS_COEF;
    obs += phys.rotMat.Dot(ball.vel - phys.vel) * VEL_COEF;

    // Statuts
    obs += player.boost / 100.f;
    obs += player.isOnGround ? 1.f : 0.f;
    obs += player.HasFlipOrJump() ? 1.f : 0.f;
    obs += player.isDemoed ? 1.f : 0.f;
    obs += player.hasJumped ? 1.f : 0.f;
    obs += player.isSupersonic ? 1.f : 0.f;
    obs += player.hasDoubleJumped ? 1.f : 0.f;
}

FList CustomObs::BuildObs(const Player& player, const GameState& state) {
    FList obs = {};

    bool inv = player.team == Team::ORANGE;

    auto ball = InvertPhys(state.ball, inv);
    auto phys = InvertPhys(player, inv);
    const auto& pads = state.GetBoostPads(inv);
    const auto& padTimers = state.GetBoostPadTimers(inv);

    // Balle (monde inversé)
    obs += ball.pos * POS_COEF;
    obs += ball.vel * VEL_COEF;
    obs += ball.angVel * ANG_VEL_COEF;

    // Heuristiques tir actuelles
    AddBallLineShots(obs, ball);

    // Dernière action
    for (int i = 0; i < player.prevAction.ELEM_AMOUNT; i++)
        obs += player.prevAction[i];

    // Boost pads global (blend via timers)
    for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
        if (pads[i]) obs += 1.f;
        else         obs += 1.f / (1.f + padTimers[i]);
    }

    // Self + extras
    FList selfObs = {};
    AddPlayerToObs(selfObs, player, inv, ball);
    AddCarBallExtras(selfObs, phys, ball, player);
    AddGoalFeatures(selfObs, phys, ball);
    AddNearestBoostFeatures(selfObs, phys, pads, padTimers, inv);
    AddFieldProximity(selfObs, phys);

    // ========== Ball prediction features ==========
    RocketSim::BallPredTracker* tracker = nullptr;
    float minDist = 1e20f, minTime = 0.f;

    // Landing
    float landT = 0.f; Vec landPos = Vec(0, 0, 0); bool foundLand = false;

    // Avantage proximité vs adversaire
    int closerCount = 0; int horizonCount = 0;

    if (state.lastArena) {
        EnsurePredTracker(state.lastArena);
        tracker = predTrackers[state.lastArena];
        if (tracker) {
            tracker->UpdatePredFromArena(state.lastArena);

            // Opposant unique (1v1)
            const Player* opp = nullptr;
            for (auto& p : state.players) {
                if (p.carId != player.carId && p.team != player.team) { opp = &p; break; }
            }
            PhysState oppPhysInv = opp ? InvertPhys(*opp, inv) : PhysState{};

            for (float t : predHorizons) {
                auto pb = tracker->GetBallStateForTime(t);
                PhysState pball = InvertPhys(pb, inv);

                // Local à l’agent
                Vec relPos = phys.rotMat.Dot(pball.pos - phys.pos);
                Vec relVel = phys.rotMat.Dot(pball.vel - phys.vel);
                selfObs += relPos * POS_COEF;
                selfObs += relVel * VEL_COEF;

                // Distance au but adverse
                Vec oppGoal(0, CommonValues::BACK_WALL_Y, 0);
                float distGoal = (oppGoal - pball.pos).Length();
                selfObs += distGoal * POS_COEF;

                // Shot on target à l’horizon
                float vy = pball.vel.y;
                float shotOpp = 0.f;
                if (std::abs(vy) > 1e-3f) {
                    float tt = (CommonValues::BACK_WALL_Y - pball.pos.y) / vy;
                    if (tt > 0) {
                        Vec hit = pball.pos + pball.vel * tt;
                        bool ok = std::abs(hit.x) <= CommonValues::GOAL_WIDTH_FROM_CENTER && hit.z > 0 && hit.z <= CommonValues::GOAL_HEIGHT;
                        shotOpp = ok ? 1.f : 0.f;
                    }
                }
                selfObs += shotOpp;

                // Min distance approx
                float d = (pball.pos - phys.pos).Length();
                if (d < minDist) { minDist = d; minTime = t; }

                // Avantage proximité vs adversaire
                if (opp) {
                    float dAgent = (pball.pos - phys.pos).Length();
                    float dOpp = (pball.pos - oppPhysInv.pos).Length();
                    closerCount += (dAgent <= dOpp) ? 1 : 0;
                }
                horizonCount++;
            }

            // Landing: première prédiction au sol
            for (size_t i = 1; i < tracker->predData.size(); i++) {
                const auto& bsCur = tracker->predData[i];
                if (bsCur.pos.z <= CommonValues::BALL_RADIUS + 5 && bsCur.vel.z <= 0) {
                    float t = static_cast<float>(i) * state.lastArena->tickTime;
                    auto bsInv = InvertPhys(bsCur, inv);
                    landT = t; landPos = bsInv.pos; foundLand = true; break;
                }
            }
        }
    }
    else {
        // Pas d'arène: on pad avec des zéros (8 scalaires par horizon + 6 finaux)
        const int scalarsPerHorizon = 8; // relPos(3) + relVel(3) + distGoal(1) + shotOpp(1)
        for (size_t i = 0; i < predHorizons.size() * scalarsPerHorizon; i++) selfObs += 0.f;
        // minDist, minTime, landX, landY, landT, proxAdv
        for (int i = 0; i < 6; i++) selfObs += 0.f;
    }

    // Ajout minDist/minTime
    selfObs += (minDist < 1e19f ? (minDist * POS_COEF) : 0.f);
    selfObs += Clamp01(minTime / 2.0f);

    // Ajout landing (x,y,t)
    if (foundLand) {
        selfObs += landPos.x * POS_COEF;
        selfObs += landPos.y * POS_COEF;
        selfObs += Clamp01(landT / 2.0f);
    }
    else {
        selfObs += 0.f; selfObs += 0.f; selfObs += 0.f;
    }

    // Avantage proximité: fraction d’horizons
    float proxAdv = (horizonCount > 0) ? (static_cast<float>(closerCount) / horizonCount) : 0.f;
    selfObs += proxAdv;

    obs += selfObs;

    // Autres joueurs (1 adversaire max)
    int playerObsSize = selfObs.size() + 9; // cohérence legacy
    std::vector<FList> teammates = {}, opponents = {};

    for (auto& otherPlayer : state.players) {
        if (otherPlayer.carId == player.carId) continue;

        auto otherPhys = InvertPhys(otherPlayer, inv);

        Vec relPos = phys.rotMat.Dot(otherPhys.pos - phys.pos);
        Vec relVel = phys.rotMat.Dot(otherPhys.vel - phys.vel);
        Vec otherAngVelLocal = phys.rotMat.Dot(otherPhys.angVel);

        FList playerObs;
        playerObs += relPos * POS_COEF;
        playerObs += relVel * VEL_COEF;
        playerObs += otherAngVelLocal * ANG_VEL_COEF;

        // Etat détaillé
        AddPlayerToObs(playerObs, otherPlayer, inv, ball);

        // Orientation adversaire dans repère de l’agent
        Vec oppFwdLocal = phys.rotMat.Dot(otherPhys.rotMat.forward);
        Vec oppUpLocal = phys.rotMat.Dot(otherPhys.rotMat.up);
        playerObs += oppFwdLocal;
        playerObs += oppUpLocal;

        // Alignement adversaire -> balle
        Vec oppToBall = (ball.pos - otherPhys.pos);
        float oppAlignToBall = otherPhys.rotMat.forward.Dot(oppToBall.Normalized());
        playerObs += oppAlignToBall;

        // Boost adversaire
        playerObs += otherPlayer.boost / 100.f;

        (otherPlayer.team == player.team ? teammates : opponents).push_back(playerObs);
    }

    if (teammates.size() > maxPlayers - 1)
        RG_ERR_CLOSE("CustomObs: Too many teammates for Obs, maximum is " << (maxPlayers - 1));
    if (opponents.size() > maxPlayers)
        RG_ERR_CLOSE("CustomObs: Too many opponents for Obs, maximum is " << maxPlayers);

    // Padding (1v1 -> 0 teammate, 1 opponent)
    for (int i = 0; i < 2; i++) {
        auto& playerList = i ? teammates : opponents;
        int targetCount = i ? maxPlayers - 1 : maxPlayers;
        while (playerList.size() < targetCount) {
            FList pad = FList(playerObsSize);
            playerList.push_back(pad);
        }
    }

    // Shuffle (inutile en 1v1, conservé pour compat)
    std::shuffle(teammates.begin(), teammates.end(), ::Math::GetRandEngine());
    std::shuffle(opponents.begin(), opponents.end(), ::Math::GetRandEngine());

    for (auto& teammate : teammates) obs += teammate;
    for (auto& opponent : opponents) obs += opponent;

    return obs;
}
