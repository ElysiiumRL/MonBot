#pragma once
#include "Reward.h"
#include "../Math.h"
#include "../CommonValues.h"

namespace RLGC {
    class EnergyReward : public Reward {
    public:
        const double GRAVITY = 650;
        const double MASS = 180;
        virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
            auto max_energy = (MASS * GRAVITY * (CommonValues::CEILING_Z - 17.)) + (0.5 * MASS * (CommonValues::CAR_MAX_SPEED * CommonValues::CAR_MAX_SPEED));
            double energy = 0;
            double velocity = player.vel.Length();

            if (player.HasFlipOrJump()) {
                energy += 0.35 * MASS * (292 * 292);
            }
            if (player.HasFlipOrJump() && !player.isOnGround) {
                double dodge_impulse = (velocity <= 1700) ? (500 + (velocity / 17)) : (600 - (velocity - 1700));
                dodge_impulse = std::max(dodge_impulse - 25, 0.0);
                //energy += 0.9 * 0.5 * MASS * (dodge_impulse * dodge_impulse);
                energy += 0.35 * MASS * 550. * 550.;
            }
            //height
            energy += MASS * GRAVITY * (player.pos.z - 17.) * 0.75;
            //KE
            energy += 0.5 * MASS * velocity * velocity;
            //boost
            energy += 7.97e5 * player.boost;
            double norm_energy = player.isDemoed ? 0.0f : (energy / max_energy);
            return norm_energy;
        }
    };
}
