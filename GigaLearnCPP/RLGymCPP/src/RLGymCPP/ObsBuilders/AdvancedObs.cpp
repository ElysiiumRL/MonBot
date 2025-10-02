#include "AdvancedObs.h"
#include <RLGymCPP/Gamestates/StateUtil.h>
#include <vector>

namespace RLGC {

    AdvancedObs::AdvancedObs(int teamSize, bool expanding)
        : teamSize(teamSize), POS_STD(2300.0f), ANG_STD((float)M_PI), expanding(expanding) {
    }

    FList AdvancedObs::BuildObs(const Player& player, const GameState& state) {
        FList obs;
        obs.reserve(237); // Reserve space for 3v3

        bool inverted = (player.team == Team::ORANGE);

        // Use the InvertPhys helper, just like in DefaultObs
        PhysState ball = InvertPhys(state.ball, inverted);
        const auto& pads = state.GetBoostPads(inverted);

        // 1. Ball data
        obs += ball.pos / POS_STD;
        obs += ball.vel / POS_STD;
        obs += ball.angVel / ANG_STD;

        // 2. Previous action from the Player object
        for (int i = 0; i < Action::ELEM_AMOUNT; i++) {
            obs += player.prevAction[i];
        }

        // 3. Boost pad states
        for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
            obs += (float)pads[i];
        }

        // 4. This player's data
        PhysState playerCar = AddPlayerToObs(obs, player, ball, inverted);

        // 5. Allies and enemies, using the DefaultObs pattern
        std::vector<const Player*> allies;
        std::vector<const Player*> enemies;
        for (const auto& other : state.players) {
            if (other.carId == player.carId) continue;
            (other.team == player.team ? allies : enemies).push_back(&other);
        }

        // 6. Add allies with padding
        int allyCount = 0;
        for (const auto* ally : allies) {
            if (allyCount >= teamSize - 1) break;
            PhysState otherCar = AddPlayerToObs(obs, *ally, ball, inverted);
            obs += (otherCar.pos - playerCar.pos) / POS_STD;
            obs += (otherCar.vel - playerCar.vel) / POS_STD;
            allyCount++;
        }
        while (allyCount < teamSize - 1) {
            AddDummy(obs);
            allyCount++;
        }

        // 7. Add enemies with padding
        int enemyCount = 0;
        for (const auto* enemy : enemies) {
            if (enemyCount >= teamSize) break;
            PhysState otherCar = AddPlayerToObs(obs, *enemy, ball, inverted);
            obs += (otherCar.pos - playerCar.pos) / POS_STD;
            obs += (otherCar.vel - playerCar.vel) / POS_STD;
            enemyCount++;
        }
        while (enemyCount < teamSize) {
            AddDummy(obs);
            enemyCount++;
        }

        return obs;
    }

    // Dummy now adds 32 floats: 26 for player data + 6 for relative data
    void AdvancedObs::AddDummy(FList& obs) {
        // 26 floats for base player data
        for (int i = 0; i < 7; i++) obs += {0, 0, 0};
        obs += {0, 0, 0, 0, 0};

        // 6 floats for extra relative data
        obs += {0, 0, 0};
        obs += {0, 0, 0};
    }

    // Helper function now correctly converted
    PhysState AdvancedObs::AddPlayerToObs(FList& obs, const Player& player, const PhysState& ball, bool inv) {
        PhysState playerCar = InvertPhys(player, inv);

        Vec relPos = ball.pos - playerCar.pos;
        Vec relVel = ball.vel - playerCar.vel;

        // Player physics data (21 floats)
        obs += relPos / POS_STD;
        obs += relVel / POS_STD;
        obs += playerCar.pos / POS_STD;
        obs += playerCar.rotMat.forward;
        obs += playerCar.rotMat.up;
        obs += playerCar.vel / POS_STD;
        obs += playerCar.angVel / ANG_STD;

        // Player state data (5 floats), using the correct GigaLearn API
        obs += FList{
            player.boost / 100.f,
            (float)player.isOnGround,
            (float)player.HasFlipOrJump(), // **FIX:** Use HasFlipOrJump() instead of hasFlip/hasJump
            (float)player.isDemoed,
            0.f // Placeholder for the removed `hasJump` to maintain size 26
        };

        return playerCar;
    }

} // namespace RLGC
