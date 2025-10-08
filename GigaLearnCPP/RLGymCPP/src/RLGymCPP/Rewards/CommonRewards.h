#pragma once
#include "Reward.h"
#include "../Math.h"

namespace RLGC {

	template<bool PlayerEventState::* VAR, bool NEGATIVE>
	class PlayerDataEventReward : public Reward {
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			bool val =  player.eventState.*VAR;

			if (NEGATIVE) {
				return -(float)val;
			} else {
				return (float)val;
			}
		}
	};

	typedef PlayerDataEventReward<&PlayerEventState::goal, false> PlayerGoalReward; // NOTE: Given only to the player who last touched the ball on the opposing team
	typedef PlayerDataEventReward<&PlayerEventState::assist, false> AssistReward;
	typedef PlayerDataEventReward<&PlayerEventState::shot, false> ShotReward;
	typedef PlayerDataEventReward<&PlayerEventState::shotPass, false> ShotPassReward;
	typedef PlayerDataEventReward<&PlayerEventState::save, false> SaveReward;
	typedef PlayerDataEventReward<&PlayerEventState::bump, false> BumpReward;
	typedef PlayerDataEventReward<&PlayerEventState::bumped, true> BumpedPenalty;
	typedef PlayerDataEventReward<&PlayerEventState::demo, false> DemoReward;
	typedef PlayerDataEventReward<&PlayerEventState::demoed, true> DemoedPenalty;

	// Rewards a goal by anyone on the team
	// NOTE: Already zero-sum
	class GoalReward : public Reward {
	public:
		float concedeScale;
		GoalReward(float concedeScale = -1) : concedeScale(concedeScale) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			if (!state.goalScored)
				return 0;

			bool scored = (player.team != RS_TEAM_FROM_Y(state.ball.pos.y));
			return scored ? 1 : concedeScale;
		}
	};

	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/misc_rewards.py
	class VelocityReward : public Reward {
	public:
		bool isNegative;
		VelocityReward(bool isNegative = false) : isNegative(isNegative) {}
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			return player.vel.Length() / CommonValues::CAR_MAX_SPEED * (1 - 2 * isNegative);
		}
	};

	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/ball_goal_rewards.py
	class VelocityBallToGoalReward : public Reward {
	public:
		bool ownGoal = false;
		int alignmentPower = 3; // can tune this

		VelocityBallToGoalReward(bool ownGoal = false, int alignmentPower = 3)
			: ownGoal(ownGoal), alignmentPower(alignmentPower) {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			bool targetOrangeGoal = player.team == Team::BLUE;
			if (ownGoal)
				targetOrangeGoal = !targetOrangeGoal;

			Vec targetPos = targetOrangeGoal ? CommonValues::ORANGE_GOAL_BACK
				: CommonValues::BLUE_GOAL_BACK;

			Vec ballDirToGoal = (targetPos - state.ball.pos).Normalized();
			float alignment = ballDirToGoal.Dot(state.ball.vel / RLGC::Math::KPHToVel(120));

			float reward = 0.0f;
			if (alignment > 0) {
				reward = powf(alignment, alignmentPower);
			}
			else {
				reward = -powf(-alignment, alignmentPower);
			}

			// --- Debug Print ---
			//std::cout << "VelocityBallToGoalReward | Team: " << (player.team == Team::BLUE ? "BLUE" : "ORANGE") << "Alignment: " << alignment << " | Reward: " << reward << std::endl;

			return reward;
		}
	};

	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/player_ball_rewards.py
	class VelocityPlayerToBallReward : public Reward {
	public:
    virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
        Vec dirToBall = (state.ball.pos - player.pos).Normalized();
        Vec normVel = player.vel / CommonValues::CAR_MAX_SPEED;
        float dot = dirToBall.Dot(normVel);
        return std::max(0.0f, dot); // clamp negative values to 0
		}
	};

	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/player_ball_rewards.py
	class FaceBallReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			Vec dirToBall = (state.ball.pos - player.pos).Normalized();
			return player.rotMat.forward.Dot(dirToBall);
		}
	};

	class TouchBallReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			return player.ballTouchedStep;
		}
	};

	class SpeedReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			return player.vel.Length() / CommonValues::CAR_MAX_SPEED;
		}
	};

	class WavedashReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			if (!player.prev)
				return 0;

			if (player.isOnGround && (player.prev->isFlipping && !player.prev->isOnGround)) {
				return 1;
			} else {
				return 0;
			}
		}
	};

	class PickupBoostReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.prev)
				return 0;

			if (player.boost > player.prev->boost) {
				return sqrtf(player.boost / 100.f) - sqrtf(player.prev->boost / 100.f);
			} else {
				return 0;
			}
		}
	};

	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/misc_rewards.py
	class SaveBoostReward : public Reward {
	public:
		float exponent;
		SaveBoostReward(float exponent = 0.5f) : exponent(exponent) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			return RS_CLAMP(powf(player.boost / 100, exponent), 0, 1);
		}
	};

	class AirReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			return !player.isOnGround;
		}
	};

	// Mostly based on the classic Necto rewards
	// Total reward output for speeding the ball up to MAX_REWARDED_BALL_SPEED is 1.0
	// The bot can do this slowly (putting) or quickly (shooting)
	class TouchAccelReward : public Reward {
	public:
		constexpr static float MAX_REWARDED_BALL_SPEED = RLGC::Math::KPHToVel(125);

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev)
				return 0;

			if (player.ballTouchedStep) {
				float prevSpeedFrac = RS_MIN(1, state.prev->ball.vel.Length() / MAX_REWARDED_BALL_SPEED);
				float curSpeedFrac = RS_MIN(1, state.ball.vel.Length() / MAX_REWARDED_BALL_SPEED);

				if (curSpeedFrac > prevSpeedFrac) {
					float reward = (curSpeedFrac - prevSpeedFrac);
					//std::cout << "[TouchAccelReward] reward=" << reward << std::endl;
					return reward;
				}
				else {
					return 0;
				}
			}
			else {
				return 0;
			}
		}
	};

	class StrongTouchReward : public Reward {
	public:
		float minRewardedVel, maxRewardedVel;
		StrongTouchReward(float minSpeedKPH = 20, float maxSpeedKPH = 120) {
			minRewardedVel = RLGC::Math::KPHToVel(minSpeedKPH);
			maxRewardedVel = RLGC::Math::KPHToVel(maxSpeedKPH);
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev)
				return 0;

			if (player.ballTouchedStep) {
				float hitForce = (state.ball.vel - state.prev->ball.vel).Length();
				if (hitForce < minRewardedVel)
					return 0;

				float reward = RS_MIN(1, hitForce / maxRewardedVel);
				//std::cout << "StrongTouchReward | Team: " << (player.team == Team::BLUE ? "BLUE" : "ORANGE") << " | HitForce: " << hitForce << " | Reward: " << reward << std::endl;
				return reward;
			}
			else {
				return 0;
			}
		}
	};

	// reward player for efficient speed
	class EnergyReward : public Reward {
	public:
		float maxEnergy;
		const int MASS = 180;
		const int GRAVITY = 650;
		const int CEILING_Z = 2044;
		const int CAR_MAX_SPEED = 2300;

		EnergyReward() {
			// Calculate max energy (potential + kinetic energy)
			maxEnergy = (MASS * GRAVITY * (CEILING_Z - 17)) + (0.5f * MASS * CAR_MAX_SPEED * CAR_MAX_SPEED);
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			float energy = 0.0f;

			// Add height potential energy (PE)
			energy += MASS * GRAVITY * (player.pos.z - 17) * 0.65f; // 0.75

			// Add kinetic energy (KE)
			Vec velocity = player.vel;
			energy += 0.5f * MASS * velocity.Dot(velocity); //energy += 0.5f * MASS * velocity.Dot(velocity); <- original

			// Add boost energy
			energy += 7.97e6f * player.boost; // 7.97e6f * player.boostFraction * 100 is original

			// Add jump energy
			if (player.HasFlipOrJump()) {
				energy += 0.35f * MASS * (292.0f * 292.0f);
			}

			if (player.HasFlipOrJump() && !player.isOnGround) {
				energy += 0.35f * MASS * 550.0f * 550.0f; //was 0.35f
			}

			// Normalize energy based on maxEnergy
			float normEnergy = energy / maxEnergy;

			// If the player is demoed, set energy to 0
			if (player.isDemoed) {
				normEnergy = 0.0f;
			}
			//std::cout << "EnergyReward | Team: " << (player.team == Team::BLUE ? "BLUE" : "ORANGE") << normEnergy << std::endl;
			return normEnergy;
		}
	};

	// reward the player for touching the ball mid air
	class JumpTouchReward : public Reward {
	public:
		JumpTouchReward(float minHeight = 600.0f, float exp = 1.0f)
			: minHeight(minHeight), exp(exp), div(std::pow(CommonValues::CEILING_Z / 2 - CommonValues::BALL_RADIUS, exp)) {
		}

		virtual void Reset(const GameState& state) override {
			ticksUntilNextReward = 0;
		};

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (player.ballTouchedStep && !player.isOnGround && state.ball.pos.z >= minHeight && ticksUntilNextReward <= 0) {
				ticksUntilNextReward = 149;
				float reward = std::pow(std::min(state.ball.pos.z, CommonValues::CEILING_Z / 2) - CommonValues::BALL_RADIUS, exp) / div;
				if (reward > 0.05) {
					//std::cout << "JumpTouchReward: " << reward << std::endl;
					return reward;
				};
			}
			ticksUntilNextReward -= 1;
			return 0.0f;
		};

	private:
		float minHeight;
		float exp;
		float div;
		int ticksUntilNextReward = 0;
	};

	// reward the player for high goal speed
	class GoalSpeedReward : public Reward {
	public:
		bool ownGoal = false;
		GoalSpeedReward(bool ownGoal = false) : ownGoal(ownGoal) {}

		virtual void Reset(const GameState& state) override {
			lastBallVel = Vec(); // reset to zero
		}

		virtual void PreStep(const GameState& state) override {
			lastBallVel = state.ball.vel;
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			constexpr static float MAX_REWARD_SPEED = RLGC::Math::KPHToVel(130);

			bool targetOrangeGoal = player.team == Team::BLUE;
			if (ownGoal)
				targetOrangeGoal = !targetOrangeGoal;

			if (state.goalScored) {

				Vec goalPos = targetOrangeGoal ? CommonValues::ORANGE_GOAL_BACK
					: CommonValues::BLUE_GOAL_BACK;

				Vec ballToGoal = (goalPos - state.ball.pos).Normalized();

				float alignment = RLGC::Math::cosine_similarity(lastBallVel, ballToGoal);

				if (alignment > 0.0f) {

					float reward = std::min(lastBallVel.Length() / MAX_REWARD_SPEED, 1.0f);

					   //std::cout << "GoalSpeedReward: " << reward << " (Speed: " << lastBallVel.Length() << ")\n";

					return reward;
				}
			}

			return 0.0f;
		}

	private:
		Vec lastBallVel;
	};

	// reward the player for having speed towards the ball mid-air
	class AerialReward2 : public Reward {
	public:
		int BALL_MIN_HEIGHT = 500;

		// Per-player latched values
		std::unordered_map<const Player*, float> anchorDistance;
		std::unordered_map<const Player*, bool> hasAnchor;

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.prev) return 0.0f;

			Vec dirToBall = (state.ball.pos - player.pos).Normalized();
			float distance = player.pos.Dist(state.ball.pos) - CommonValues::BALL_RADIUS;

			const float MIN_DIST = 2300.0f;

			float speedTowardBall = player.vel.Dot(dirToBall);
			float speedReward = std::clamp(speedTowardBall / CommonValues::CAR_MAX_SPEED, 0.0f, 1.0f);

			// Early exits / reset
			if (player.isOnGround || state.ball.pos.z < BALL_MIN_HEIGHT || player.boost < 1 || speedTowardBall < 0) {
				hasAnchor[&player] = false;
				return 0.0f;
			}

			// Snapshot the anchor ONCE when starting the aerial
			if (!hasAnchor[&player]) {
				anchorDistance[&player] = distance;
				hasAnchor[&player] = true;
			}

			float anchor = anchorDistance[&player];

			float distanceRatio = std::clamp(1.0f - distance / MIN_DIST, 0.0f, 1.0f);
			float anchorDistanceRatio = std::clamp(1.0f - anchor / MIN_DIST, 0.0f, 1.0f);

			float reward = 0.0f;
			if (distanceRatio > anchorDistanceRatio) {
				reward = (distanceRatio - anchorDistanceRatio) * speedReward;
			}

			if (player.hasDoubleJumped)
				reward *= 2.f;

			// Debug: uncomment to see stable anchor vs current
			// std::cout << "Anchor: " << anchor << " | Current: " << distance << " | RatioDiff: " << (distanceRatio - anchorDistanceRatio) << " | Reward: " << reward << std::endl;

			return reward;
		}
	};

	class LemTouchBallReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// Check if the player touched the ball
			if (player.ballTouchedStep) {
				// Check if the player is in the air and above height 256
				if (!player.isOnGround && player.pos.z >= 256) {
					// Compute height reward using log1p (log(1 + x))
					return std::log1p(player.pos.z - 256);
				}
			}
			return 0.0f;
		}
	};

	class AlignmentDistScaled : public Reward {
	public:
		constexpr static float DIST_SCALE = 1410.0f;  // distance scale, max driving speed

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			Vec ball_pos = state.ball.pos;
			Vec pos = player.pos;

			// Define goal positions
			Vec blueGoalBack = CommonValues::BLUE_GOAL_BACK;
			Vec orangeGoalBack = CommonValues::ORANGE_GOAL_BACK;

			float closestDist = std::numeric_limits<float>::max();
			const Player* closestPlayer = nullptr;

			for (const auto& other : state.players) {
				if (other.team != player.team) continue;  // only teammates
				float d = (ball_pos - other.pos).Length();
				if (d < closestDist) {
					closestDist = d;
					closestPlayer = &other;
				}
			}

			// Only reward the closest teammate
			if (closestPlayer == nullptr || closestPlayer->carId != player.carId)
				return 0.0f;

			float alignment = 0.5f * (
				RLGC::Math::cosine_similarity(ball_pos - pos, orangeGoalBack - pos) -
				RLGC::Math::cosine_similarity(ball_pos - pos, blueGoalBack - pos)
				);

			if (player.team == Team::ORANGE) {
				alignment *= -1.0f;
			}

			// Distance decay
			float dist = (ball_pos - pos).Length();
			float scale = expf(-dist / DIST_SCALE);  // between 0 and 1

			alignment *= scale;

			return alignment;
		}
	};

	// reward bot for scoring higher goals
	class GoalHeightReward : public Reward {
	public:
		bool ownGoal = false;
		GoalHeightReward(bool ownGoal = false) : ownGoal(ownGoal) {}

		virtual void Reset(const GameState& state) override {
			lastBallPos = Vec();
		}

		virtual void PreStep(const GameState& state) override {
			lastBallPos = state.ball.pos;
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			const float MAX_GOAL_HEIGHT = CommonValues::GOAL_HEIGHT - (CommonValues::BALL_RADIUS * 2.f);

			bool targetOrangeGoal = player.team == Team::BLUE;
			if (ownGoal)
				targetOrangeGoal = !targetOrangeGoal;

			if (state.goalScored) {
				Vec goalPos = targetOrangeGoal ? CommonValues::ORANGE_GOAL_BACK : CommonValues::BLUE_GOAL_BACK;

				Vec ballToGoal = (goalPos - lastBallPos).Normalized();
				float alignment = RLGC::Math::cosine_similarity(state.ball.vel, ballToGoal);

				if (alignment > 0.0f) {
					float reward = std::min(lastBallPos.z / MAX_GOAL_HEIGHT, 1.0f);
					//std::cout << "GoalHeightReward: " << reward << " (Height: " << lastBallPos.z << ")\n";
					return reward;
				}
			}

			return 0.0f;
		}

	private:
		Vec lastBallPos;
	};

	class ClosestKickoffReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			if (!state.ball.vel.IsZero()) {
				return 0;
			}

			const Player* closestPlayer = nullptr;
			//const Player* farthestPlayer = nullptr;
			float minDistance = std::numeric_limits<float>::max();

			// Find the closest and farthest players
			for (const auto& p : state.players) {
				float distance = (p.pos - state.ball.pos).Length();
				if (distance < minDistance) {
					minDistance = distance;
					closestPlayer = &p;
				}

			}

			if (closestPlayer && closestPlayer->carId == player.carId) {
				return 1.0f;
			}

			return 0.0f;  // Others get 0
		}
	};

	class DribbleReward : public Reward {
	public:
		float minRewardedVel, maxRewardedVel;

		DribbleReward(float minSpeedKPH = 30, float maxSpeedKPH = 135) {
			minRewardedVel = RLGC::Math::KPHToVel(minSpeedKPH);
			maxRewardedVel = RLGC::Math::KPHToVel(maxSpeedKPH);
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			const float MIN_BALL_HEIGHT = 101.0f;
			const float MAX_BALL_HEIGHT = 180.0f;
			const float MAX_DISTANCE = 196.0f;
			const float SPEED_MATCH_FACTOR = 1.9f;
			const float MAX_SPEED = 2050.0f;

			const Vec ORANGE_GOAL_CENTER(0, 5120, 457);
			const Vec BLUE_GOAL_CENTER(0, -5120, 457);
			Vec opponentGoal = (player.team == Team::BLUE ? ORANGE_GOAL_CENTER : BLUE_GOAL_CENTER);

			const Player* closestPlayer = nullptr;
			float minDistance = std::numeric_limits<float>::max();
			for (const auto& p : state.players) {
				float distance = (p.pos - state.ball.pos).Length();
				if (distance < minDistance) {
					minDistance = distance;
					closestPlayer = &p;
				}
			}

			if (player.isOnGround &&
				state.ball.pos.z >= MIN_BALL_HEIGHT && state.ball.pos.z <= MAX_BALL_HEIGHT &&
				(player.pos - state.ball.pos).Length() < MAX_DISTANCE && closestPlayer) {

				float playerSpeed = player.vel.Length();
				float ballSpeed = state.ball.vel.Length();

				float normalizedPlayerSpeed = playerSpeed / MAX_SPEED;

				float speedMatchReward = normalizedPlayerSpeed +
					SPEED_MATCH_FACTOR * (1.0f - std::abs(playerSpeed - ballSpeed) / (playerSpeed + ballSpeed + 1.0f));

				Vec toGoal = (opponentGoal - player.pos).Normalized();
				float velocityTowardsGoal = player.vel.Dot(toGoal);
				float goalDirectionScale = std::max(0.0f, velocityTowardsGoal / MAX_SPEED);

				float GOAL_DIRECTION_EXP = 2.0f; // higher = stronger top-end emphasis
				float shapedGoalDir = std::pow(goalDirectionScale, GOAL_DIRECTION_EXP);

				float finalReward = speedMatchReward * goalDirectionScale; //could use shapedGoalDir

			//	float bonus = 0.0f; //player.prev->lastControls.yaw != 0 &&
				//if (player.isJumping && player.isFlipping && player.flipRelTorque.y < 0 &&
				//	player.flipRelTorque.x > -1 && player.flipRelTorque.x < 1 && player.prev->lastControls.yaw != 0 &&
				//	state.ball.pos.z > player.pos.z && player.ballTouchedStep) {

					//float velIncrease = (state.ball.vel - state.prev->ball.vel).Length();
					//if (velIncrease >= minRewardedVel) {
					//	bonus = RS_MIN(1.0f, velIncrease / maxRewardedVel) * goalDirectionScale;
					//}
				//}

				return finalReward;

			}

			return 0.0f;
		}
	};

	class AirDribbleTest : public Reward {
	public:
		const float MIN_BALL_DIST = 400.0f;
		const float MIN_BOOST = 15.0f;
		const float MIN_HEIGHT = 300.0f;
		const float MAX_BALL_SPEED = 2500.0f;
		const float MAX_RELATIVE_SPEED = 475.0f;
		const float MIN_RELATIVE_SPEED = 150.0f;

		const float RAMP_HEIGHT = 256.0f;
		const float MAX_HEIGHT = 2000.0f;
		const float HEIGHT_WEIGHT = 0.5f;
		const float FLIP_RESET_MULT = 5.0f;

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			float carZ = player.pos.z;
			float ballZ = state.ball.pos.z;

			if (!player.ballTouchedStep)
				return 0.0f;

			if (player.isOnGround)
				return 0.0f;

			if (player.boost < MIN_BOOST)
				return 0.0f;

			if (player.pos.z < MIN_HEIGHT)
				return 0.0f;

			if (player.pos.z >= state.ball.pos.z)
				return 0.0f;

			if (player.rotMat.forward.z <= 0.3f)
				return 0.0f;

			if (player.isFlipping)
				return 0.0f;

			if (carZ > ballZ - 150.0f) return 0.0f;

			if ((state.ball.pos - player.pos).Length() >= MIN_BALL_DIST)
				return 0.0f;

			const Vec ORANGE_GOAL_CENTER(0, 5120, 450);
			const Vec BLUE_GOAL_CENTER(0, -5120, 450);
			Vec targetGoal = player.team == Team::BLUE ? ORANGE_GOAL_CENTER : BLUE_GOAL_CENTER;

			Vec toGoal = (targetGoal - state.ball.pos).Normalized();
			float ballTowardGoal = state.ball.vel.Dot(toGoal);

			float normalizedBallTowardGoal = ballTowardGoal / MAX_BALL_SPEED;
			if (normalizedBallTowardGoal < 0.0f) normalizedBallTowardGoal = 0.0f;
			if (normalizedBallTowardGoal > 1.0f) normalizedBallTowardGoal = 1.0f;

			Vec relativeVel = player.vel - state.ball.vel;
			float relSpeed = relativeVel.Length();

			if (relSpeed < MIN_RELATIVE_SPEED)
				return 0.0f;

			if (relSpeed > MAX_RELATIVE_SPEED)
				relSpeed = MAX_RELATIVE_SPEED;

			float normalizedRelSpeed = relSpeed / MAX_RELATIVE_SPEED;

			float reward = player.airTime * normalizedBallTowardGoal; // * normalizedRelSpeed and norm goal speed

			if (player.HasFlipReset()) {
				reward *= FLIP_RESET_MULT;
			}

			// height bonus
			float rawHeightValue = (player.pos.z + state.ball.pos.z) - 2.0f * RAMP_HEIGHT;
			float normalizedHeight = rawHeightValue / (2.0f * (MAX_HEIGHT - RAMP_HEIGHT));
			if (normalizedHeight < 0.0f) normalizedHeight = 0.0f;
			if (normalizedHeight > 1.0f) normalizedHeight = 1.0f;

			//reward += HEIGHT_WEIGHT * normalizedHeight;

			//std::cout << "AirDribbleTest | Team: " << (player.team == Team::BLUE ? "BLUE" : "ORANGE") << " | BallDist: " << (state.ball.pos - player.pos).Length() << " | RelSpeed: " << relSpeed << " | BallTowardGoal: " << ballTowardGoal << " | NormalizedHeight: " << normalizedHeight << " | AirTime: " << player.airTime << " | FlipReset: " << (player.HasFlipReset() ? "YES" : "NO") << " | Reward: " << reward << std::endl;

			return reward;
		}
	};

	class AirDribbleReward2 : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {

			bool touchedBall = player.ballTouchedStep;
			bool airborne = !player.isOnGround;
			bool facingUp = player.rotMat.forward.z > 0;
			bool highEnough = player.pos.z > 300.f;
			bool fastEnough = player.vel.Length() > 300.f;
			const float MAX_SPEED = 2200.0f;
			const Vec ORANGE_GOAL_CENTER(0, 5120, 512);
			const Vec BLUE_GOAL_CENTER(0, -5120, 512);
			Vec opponentGoal = (player.team == Team::BLUE ? ORANGE_GOAL_CENTER : BLUE_GOAL_CENTER);

			if (!(touchedBall && airborne && facingUp && highEnough && fastEnough))
				return 0.0f;

			Vec ball_to_goal = (player.team == Team::BLUE ? CommonValues::ORANGE_GOAL_CENTER : CommonValues::BLUE_GOAL_CENTER) - state.ball.pos;

			float rawAlignment = 0.5f * (RLGC::Math::cosine_similarity(state.ball.vel, ball_to_goal) -
				RLGC::Math::cosine_similarity(state.ball.vel, -ball_to_goal));

			float alignment = std::clamp(rawAlignment, 0.0f, 1.0f);

			Vec distanceToGoal = (opponentGoal - player.pos).Normalized();
			float velocityTowardsGoal = player.vel.Dot(distanceToGoal);
			float goalDirectionScale = std::max(0.0f, velocityTowardsGoal / MAX_SPEED);

			float consecutiveReward = 1.0f;

			return consecutiveReward * alignment * goalDirectionScale;
		}
	};

	class SimpleAirDribbleReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.prev)
				return 0.0f;

			bool touchedBall = player.ballTouchedStep;
			bool airborne = !player.isOnGround;
			bool enoughBoost = player.boost >= 20.f;
			bool notFlipping = !player.isFlipping;
			bool ballHigh = state.ball.pos.z > 200.f;
			bool playerUnderBall = player.pos.z < (state.ball.pos.z - 70.f);
			bool correctOrientation = player.rotMat.forward.z >= 0.f;

			Vec relVel = player.vel - state.ball.vel;
			bool relVelCheck = relVel.Length() <= 400.f;

			if (!(touchedBall && airborne && enoughBoost && notFlipping && ballHigh && playerUnderBall && relVelCheck && correctOrientation))
				return 0.0f;

			const Vec ORANGE_GOAL_CENTER(0, 5120, 450);
			const Vec BLUE_GOAL_CENTER(0, -5120, 450);
			Vec targetGoal = player.team == Team::BLUE ? ORANGE_GOAL_CENTER : BLUE_GOAL_CENTER;
			Vec toGoal = (targetGoal - state.ball.pos).Normalized();

			float ballTowardGoal = state.ball.vel.Dot(toGoal);

			const float MAX_BALL_SPEED = 2300.0f;
			float normalizedBallTowardGoal = ballTowardGoal / MAX_BALL_SPEED;
			if (normalizedBallTowardGoal < 0.0f) normalizedBallTowardGoal = 0.0f;
			if (normalizedBallTowardGoal > 1.0f) normalizedBallTowardGoal = 1.0f;

			//std::cout << "[AirDribbleReward] carID=" << player.carId << " reward=" << normalizedBallTowardGoal << " ballZ=" << state.ball.pos.z << " playerZ=" << player.pos.z << " boost=" << player.boost << " relVel=" << relVel.Length() << std::endl;

			return normalizedBallTowardGoal;
		}
	};

	inline float solid_angle_eriksson(const Vec& O, const Vec& A, const Vec& B, const Vec& C) {
		// Calculate the solid angle of a triangle ABC from the point O
		Vec a = A - O;
		Vec b = B - O;
		Vec c = C - O;

		// Normalize vectors
		a = a.Normalized();
		b = b.Normalized();
		c = c.Normalized();

		// Calculate cross product of a and b
		Vec crossAB = a.Cross(b);

		// Calculate dot product of crossAB and c
		float numerator = crossAB.Dot(c);

		// Take absolute value for the numerator
		numerator = std::abs(numerator);

		// Calculate denominator
		float denominator = 1.0f + a.Dot(b) + b.Dot(c) + c.Dot(a);

		// Calculate the solid angle
		float E = 2.0f * std::atan2(numerator, denominator);

		return E;
	}

	inline float view_goal_ratio(const Vec& pos, float goal_y, float margin = CommonValues::BALL_RADIUS) {
		/*
			GOAL_HEIGHT = 642.775  # uu
			GOAL_CENTER_TO_POST = 892.755  # uu*/
			//	Calculate the percent of the field of view that the goal takes up
		float max_x = 892.755f - margin;
		float min_x = -max_x;
		float max_z = 642.775f - margin;
		float min_z = margin;

		// Define the four corners of the goal
		Vec bl(min_x, goal_y, min_z);  // Bottom left
		Vec br(max_x, goal_y, min_z);  // Bottom right
		Vec tl(min_x, goal_y, max_z);  // Top left
		Vec tr(max_x, goal_y, max_z);  // Top right

		// Calculate solid angles for the two triangles making up the goal rectangle
		float solid_angle_1 = solid_angle_eriksson(pos, bl, br, tl);
		float solid_angle_2 = solid_angle_eriksson(pos, br, tr, tl);

		// Return the ratio of the solid angle to the full sphere (4p)
		return (solid_angle_1 + solid_angle_2) / (4.0f * M_PI);
	}

	class GoalProbReward : public Reward {
	public:
		GoalProbReward(float gamma = 0.99525f) : gamma(gamma), prob(0.0f) {}

		virtual void Reset(const GameState& state) override {
			prob = CalculateBlueGoalProb(state);
		}


		virtual std::vector<float> GetAllRewards(const GameState& state, bool isFinal) override {
			float currentProb = CalculateBlueGoalProb(state);

			// Probability goes from 0-1, but for a reward we want it to go from -1 to 1
			// 2x-1 - (2y-1) = 2(x-y)
			float reward = 2.0f * (gamma * currentProb - prob);

			// Store current probability for next step
			prob = currentProb;

			std::vector<float> rewards = std::vector<float>(state.players.size());
			for (int i = 0; i < state.players.size(); i++) {
				rewards[i] = state.players[i].team == Team::BLUE ? reward : -reward;
			}

			return rewards;
		}


	protected:
		// This is a virtual method to be implemented by derived classes
		virtual float CalculateBlueGoalProb(const GameState& state) {
			// Base implementation - should be overridden
			return 0.0f;
		}

	private:
		float gamma;
		float prob;
	};

	class GoalViewReward : public GoalProbReward {
	public:
		GoalViewReward(float gamma = 1.0f) : GoalProbReward(gamma) {}

	protected:
		// Override the calculation method with the solid angle implementation
		virtual float CalculateBlueGoalProb(const GameState& state) override {
			const Vec& ballPos = state.ball.pos;

			// Calculate view ratios for both goals
			// Negative GOAL_THRESHOLD for blue net (orange scoring)
			float viewBlue = view_goal_ratio(ballPos, -5215.5);

			// Positive GOAL_THRESHOLD for orange net (blue scoring)
			float viewOrange = view_goal_ratio(ballPos, 5215.5);

			// Return probability of blue scoring (hitting orange goal)
			float result = viewOrange / (viewBlue + viewOrange);
			//std::cout << "GoalViewReward: " << result << std::endl;
			return result;
		}
	};

	class LowSpeedPunish : public Reward {
	public:
		float SPEED_THRESHOLD = 500.0f;

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			float speed = player.vel.Length();

			// If player has flipped and is slow, apply flat -1 penalty
			//if (player.hasFlipped && speed < 400.0f) {
			//	return -1.0f;
			//}

			// Otherwise apply normal low-speed penalty
			if (speed < SPEED_THRESHOLD) {
				// bigger penalty the slower you are
				//return -(SPEED_THRESHOLD - speed) / SPEED_THRESHOLD;
				return -1;
			}

			return 0.0f;
		}
	};

	class AerialBoostReward : public Reward {
	public:
		const float MIN_BALL_DIST = 400.0f;
		const float MIN_VELOCITY = 100.0f;

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			// Direction to ball
			Vec dirToBall = (state.ball.pos - player.pos).Normalized();

			// Alignment: facing ball means forward vector dot dirToBall close to 1
			float alignment = player.rotMat.forward.Dot(dirToBall);

			// Only reward if all requirements are met
			if (alignment > 0.75f &&
				!player.isOnGround &&                     // in air
				player.boost < player.prev->boost &&       // using boost
				(state.ball.pos - player.pos).Length() < MIN_BALL_DIST &&
				player.pos.z < state.ball.pos.z &&
				player.rotMat.forward.z > 0.5) {

				// Project velocity onto direction to ball (speed toward ball)
				float speedTowardBall = player.vel.Dot(dirToBall);

				// Normalize by car max speed so it stays within [0, 1]
				return speedTowardBall / CommonValues::CAR_MAX_SPEED;
			}

			return 0.0f; // otherwise no reward
		}
	};

	class TeamSpacingReward : public Reward {
	public:
		const float MIN_SPACING = 150.0f;
		const float MAX_SPACING = 900.0f;

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			float penalty = 0.0f;
			if (state.ball.vel.IsZero()) {
				return 0.0f;
			}

			for (const auto& other : state.players) {
				if (&other == &player)
					continue; // skip self
				if (other.team != player.team)
					continue; // only check teammates

				float dist = (other.pos - player.pos).Length();

				if (dist < MAX_SPACING) {
					// -1.0f at min, 0.0f at max
					float t = (dist - MIN_SPACING) / (MAX_SPACING - MIN_SPACING);
					t = std::clamp(t, 0.0f, 1.0f);
					float scaledPenalty = -1.0f * (1.0f - t);

					// Apply the strongest penalty (if multiple teammates)
					penalty = std::min(penalty, scaledPenalty);
				}
			}

			return penalty;
		}
	};


	// Punish conceding player for being far from the ball when a goal is scored
	class GoalDistancePunish : public Reward {
	public:
		GoalDistancePunish() {}

		virtual void Reset(const GameState& state) override {}

		virtual void PreStep(const GameState& state) override {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.goalScored)
				return 0.0f;

			// Determine if this player scored or conceded
			bool scored = (player.team != RS_TEAM_FROM_Y(state.ball.pos.y));

			if (scored)
				return 0.0f;  // only punish conceding players

			// Player is on the conceding team, compute distance
			float dist = (player.pos - state.ball.pos).Length();

			constexpr float CAR_MAX_SPEED = 2300.0f;
			float scaled = 1.0f - std::exp(-dist / CAR_MAX_SPEED);

			float punish = -scaled;

			//std::cout << "GoalConcedeDistancePunish: player " << player.carId
				//<< " distance " << dist
				//<< " scaled punish " << punish << std::endl;

			return punish;
		}
	};

	// Reward for touching the ball higher up and farther from walls
	class TouchHeightReward : public Reward {
	public:
		const float CEILING_Z = 2044.0f;
		const float CAR_MAX_SPEED = 2300.0f;
		constexpr static float MAX_REWARDED_BALL_SPEED = RLGC::Math::KPHToVel(105);

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.ballTouchedStep || !state.prev)
				return 0.0f;

			// --- Height Factor ---
			float avgHeight = 0.5f * (player.pos.z + state.ball.pos.z);
			auto heightActivation = [&](float z) {
				return cbrtf((z - 150.0f) / CEILING_Z);
				};
			float h0 = heightActivation(0.0f);
			float h1 = heightActivation(CEILING_Z);
			float hx = heightActivation(avgHeight);
			float heightFactor = (hx - h0) / (h1 - h0);
			heightFactor = heightFactor * heightFactor;

			// --- Wall Distance Factor ---
			float distToWall = DistToClosestWall(player.pos.x, player.pos.y);
			float wallDistFactor = 1.0f - expf(-distToWall / CAR_MAX_SPEED);

			float baseReward = heightFactor * (1.0f + wallDistFactor);

			// --- Ball Speed Scaling ---
			float prevSpeedFrac = RS_MIN(1, state.prev->ball.vel.Length() / MAX_REWARDED_BALL_SPEED);
			float curSpeedFrac = RS_MIN(1, state.ball.vel.Length() / MAX_REWARDED_BALL_SPEED);

			// --- Ball Speed Toward Goal Scaling ---
			const Vec ORANGE_GOAL_CENTER(0, 5120, 450);
			const Vec BLUE_GOAL_CENTER(0, -5120, 450);
			Vec targetGoal = player.team == Team::BLUE ? ORANGE_GOAL_CENTER : BLUE_GOAL_CENTER;

			Vec toGoal = (targetGoal - state.ball.pos).Normalized();
			float ballTowardGoal = state.ball.vel.Dot(toGoal);

			float normalizedBallTowardGoal = ballTowardGoal / MAX_REWARDED_BALL_SPEED;
			if (normalizedBallTowardGoal < 0.0f) normalizedBallTowardGoal = 0.0f;
			if (normalizedBallTowardGoal > 1.0f) normalizedBallTowardGoal = 1.0f;

			if (curSpeedFrac > prevSpeedFrac) {
				float speedScale = curSpeedFrac - prevSpeedFrac;
				float reward = baseReward * speedScale;

				//std::cout << "TouchHeightReward | Team: " << (player.team == Team::BLUE ? "BLUE" : "ORANGE") << " | HeightFactor: " << heightFactor << " | WallDist: " << distToWall << " | SpeedChange: " << speedScale << " | Reward: " << reward << std::endl;

				return reward;
			}
			else {
				return 0.0f;
			}
		}

	private:
		float DistToClosestWall(float x, float y) const {
			float distSideWall = fabsf(4096.0f - fabsf(x));
			float distBackWall = fabsf(5120.0f - fabsf(y));
			return std::min(distSideWall, distBackWall);
		}
	};

	class AFKTrollPenalty : public Reward {
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			float reward = 0.0f;
			if (player.rotMat.up.z < 0.0f && player.pos.z < 50.0f && player.vel.Length() < 200.0f) {
				reward += -1.0f;
			}
			return reward;
		}
	};

	class EngagedDistanceReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {

			float reward = 0.0f;

			float LIMIT_DISTANCE = 5000.0f;
			float distFromBall = (player.pos.y - state.ball.pos.y);
			if (distFromBall > LIMIT_DISTANCE) {
				reward += -(std::max((distFromBall - LIMIT_DISTANCE) / (10240.0f - LIMIT_DISTANCE), 0.0f));
			}

			//if (reward != 0.0f) { std::cout << "EngagedDistanceReward | Player " << player.carId << " | DistFromBall: " << distFromBall << " | Reward: " << reward << std::endl; }

			return reward;
		}
	};


	class PossessionReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev)
				return 0.0f;

			constexpr float CAR_RADIUS = 73.13f; // Octane value here
			constexpr float BALL_RADIUS = 92.75f;
			constexpr float CONTACT_DIST = CAR_RADIUS + BALL_RADIUS;

			// Distance from player front hitbox edge to ball surface
			float myDist = (state.ball.pos - player.pos).Length() - CONTACT_DIST;

			// Find closest opponent
			float closestOpponentDist = std::numeric_limits<float>::max();
			for (auto& other : state.players) {
				if (other.team == player.team)
					continue;

				float oppDist = (state.ball.pos - other.pos).Length() - CONTACT_DIST;
				if (oppDist < closestOpponentDist)
					closestOpponentDist = oppDist;
			}

			if (closestOpponentDist == std::numeric_limits<float>::max())
				return 0.0f;

			return myDist < closestOpponentDist ? 1.0f : 0.0f;
		}
	};

	class DribbleBumpReward : public Reward {
	public:
		float dribbleDistance;
		float carBallHeightDiff;
		float maxTimeSinceDribble;
		float baseReward;
		float speedBonus;

		std::unordered_map<uint32_t, bool> isDribbling;
		std::unordered_map<uint32_t, float> timeSinceDribble;
		std::unordered_map<uint32_t, float> lastDribbleSpeed;

		DribbleBumpReward(
			float dribbleDistance = 197.0f,
			float carBallHeightDiff = 101.0f,
			float maxTimeSinceDribble = 1.0f,
			float baseReward = 1.0f,
			float speedBonus = 0.5f
		) : dribbleDistance(dribbleDistance),
			carBallHeightDiff(carBallHeightDiff),
			maxTimeSinceDribble(maxTimeSinceDribble),
			baseReward(baseReward),
			speedBonus(speedBonus) {
		}

		virtual void Reset(const GameState& initialState) override {
			isDribbling.clear();
			timeSinceDribble.clear();
			lastDribbleSpeed.clear();

			for (const auto& player : initialState.players) {
				isDribbling[player.carId] = false;
				timeSinceDribble[player.carId] = 999.0f;
				lastDribbleSpeed[player.carId] = 0.0f;
			}
		}

		virtual void PreStep(const GameState& state) override {
			for (const auto& player : state.players) {
				uint32_t carId = player.carId;

				Vec ballCarVec = state.ball.pos - player.pos;
				bool isCurrentlyDribbling = (
					ballCarVec.Length() < dribbleDistance &&
					state.ball.pos.z > (player.pos.z + carBallHeightDiff)
					);

				if (isDribbling[carId] && !isCurrentlyDribbling) {
					timeSinceDribble[carId] = 0.0f;
					lastDribbleSpeed[carId] = player.vel.Length();
				}
				else if (isCurrentlyDribbling) {
					timeSinceDribble[carId] = 999.0f;
				}
				else if (timeSinceDribble[carId] < 999.0f) {
					timeSinceDribble[carId] += state.deltaTime;
				}

				isDribbling[carId] = isCurrentlyDribbling;
			}
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			uint32_t carId = player.carId;

			if (!player.eventState.bump) {
				return 0.0f;
			}

			if (timeSinceDribble[carId] > maxTimeSinceDribble) {
				return 0.0f;
			}

			float reward = baseReward;

			float timeFactor = 1.0f - (timeSinceDribble[carId] / maxTimeSinceDribble);
			reward *= (1.0f + timeFactor * 0.5f);

			float speedFactor = lastDribbleSpeed[carId] / CommonValues::CAR_MAX_SPEED;
			reward *= (1.0f + speedFactor * speedBonus);

			float ballDistance = (state.ball.pos - player.pos).Length();
			if (ballDistance < dribbleDistance * 1.5f) {
				reward *= 1.25f;
			}

			if (player.eventState.demo) {
				reward *= 1.5f;
			}

			return RS_CLAMP(reward, 0.0f, 3.0f);
		}
	};



}
