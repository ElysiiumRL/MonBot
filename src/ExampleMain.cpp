#include <GigaLearnCPP/Learner.h>

#include <RLGymCPP/Rewards/CommonRewards.h>
#include <RLGymCPP/Rewards/ZeroSumReward.h>
#include <RLGymCPP/TerminalConditions/NoTouchCondition.h>
#include <RLGymCPP/TerminalConditions/GoalScoreCondition.h>
#include <RLGymCPP/ObsBuilders/DefaultObs.h>
#include <RLGymCPP/ObsBuilders/AdvancedObs.h>
#include <RLGymCPP/ObsBuilders/CustomObs.h>
#include <RLGymCPP/StateSetters/KickoffState.h>
#include <RLGymCPP/StateSetters/RandomState.h>
#include <RLGymCPP/StateSetters/CombinedState.h>
#include <RLGymCPP/ActionParsers/DefaultAction.h>

using namespace GGL; // GigaLearn
using namespace RLGC; // RLGymCPP

// Create the RLGymCPP environment for each of our games
EnvCreateResult EnvCreateFunc(int index) {
	std::vector<WeightedReward> rewards = {
	
		// Movement
				{ new AirReward(), 0.15f },
				{ new EnergyReward(), 0.005f },
		
				// Player-ball
				{ new FaceBallReward(), 0.2f },
				{ new VelocityPlayerToBallReward(), 3.f },
				{ new ZeroSumReward(new StrongTouchReward(20, 110), 1, 0.0f), 50 },
				{ new ZeroSumReward(new TouchHeightReward(), 1), 30 },
				{ new ZeroSumReward(new AerialReward2(), 1, 1), 60.f },
		
		
				// Ball-goal
				{ new VelocityBallToGoalReward(), 2.f },
		
				// Boost
				{ new ZeroSumReward(new PickupBoostReward(), 0.3f, 1), 45.f },
				{ new ZeroSumReward(new SaveBoostReward(), 0, 0.25f), 0.2f },
		
				// Game events
				{ new ZeroSumReward(new BumpReward(), 0.5f), 80 },
				{ new ZeroSumReward(new DemoReward(), 0.5f), 120 },
				{ new ZeroSumReward(new GoalSpeedReward(), 1, 0), 125 },
				{ new GoalReward(), 300.f },
			};

	bool visualizeMode = false; // enable to true to override player count and visualize 2s
	int visualizerPlayerCount = 2;
	int playersPerTeam;

	if (visualizeMode) {
		playersPerTeam = visualizerPlayerCount;
		}
	else {
		int pattern = index % 9;
		if (pattern < 6)
			playersPerTeam = 1; // split so that 1s gives the same effective experience as 2s
		else
			playersPerTeam = 1;
		}

	//int playersPerTeam = 1; //index % 3 + 1;
	auto arena = Arena::Create(GameMode::SOCCAR);
	for (int i = 0; i < playersPerTeam; i++) {
		arena->AddCar(Team::BLUE);
		arena->AddCar(Team::ORANGE);
	}

	EnvCreateResult result = {};
	result.actionParser = new DefaultAction();
	result.obsBuilder = new CustomObs();
	result.stateSetter = new CombinedState(
		{
			{new KickoffState(), 0.75f},
			{new RandomState(true, true, false), 0.375f},
		}
		);
	result.terminalConditions = terminalConditions;
	result.rewards = rewards;

	result.arena = arena;

	return result;
}

void StepCallback(Learner* learner, const std::vector<GameState>& states, Report& report) {
	// To prevent expensive metrics from eating at performance, we will only run them on 1/4th of steps
	// This doesn't really matter unless you have expensive metrics (which this example doesn't)
	bool doExpensiveMetrics = (rand() % 4) == 0;

	// Add our metrics
	for (auto& state : states) {
		if (doExpensiveMetrics) {
			for (auto& player : state.players) {
				report.AddAvg("Player/In Air Ratio", !player.isOnGround);
				report.AddAvg("Player/Ball Touch Ratio", player.ballTouchedStep);
				report.AddAvg("Player/Demoed Ratio", player.isDemoed);
				report.AddAvg("Player/DoubleJumped Ratio", player.hasDoubleJumped);
				report.AddAvg("Player/Supersonic Ratio", player.isSupersonic);
				report.AddAvg("Player/Flip Reset Ratio", player.HasFlipReset());

				report.AddAvg("Player/Speed", player.vel.Length());
				Vec dirToBall = (state.ball.pos - player.pos).Normalized();
				report.AddAvg("Player/Speed Towards Ball", RS_MAX(0, player.vel.Dot(dirToBall)));

				report.AddAvg("Player/Boost", player.boost);

				if (player.ballTouchedStep)
					report.AddAvg("Player/Touch Height", state.ball.pos.z);
			}
		}

		if (state.goalScored)
			report.AddAvg("Game/Goal Speed", state.ball.vel.Length());
	}
}

int main(int argc, char* argv[]) {
	RocketSim::Init("/workspace/bot/collision_meshes");

	LearnerConfig cfg = {};
	cfg.deviceType = LearnerDeviceType::GPU_CUDA;

	cfg.tickSkip = 8;
	cfg.actionDelay = cfg.tickSkip - 1; // Normal value in other RLGym frameworks

	// Play around with this to see what the optimal is for your machine, more games will consume more RAM
	cfg.numGames = 500; //256 default

	// Leave this empty to use a random seed each run
	// The random seed can have a strong effect on the outcome of a run
	cfg.randomSeed = 123;

	int tsPerItr = 100'000;
	cfg.ppo.tsPerItr = tsPerItr;
	cfg.ppo.batchSize = tsPerItr;
	cfg.ppo.miniBatchSize = 50'000; // Lower this if too much VRAM is being allocated

	// Using 2 epochs seems pretty optimal when comparing time training to skill
	// Perhaps 1 or 3 is better for you, test and find out!
	cfg.ppo.epochs = 1;

	// This scales differently than "ent_coef" in other frameworks
	// This is the scale for normalized entropy, which means you won't have to change it if you add more actions
	cfg.ppo.entropyScale = 0.035f; //0.035

	// Rate of reward decay
	// Starting low tends to work out
	cfg.ppo.gaeGamma = 0.991;

	// Good learning rate to start
	cfg.ppo.policyLR = 2e-4;
	cfg.ppo.criticLR = 2e-4;

	cfg.ppo.sharedHead.layerSizes = { 2048, 1024, 1024,};
        cfg.ppo.policy.layerSizes = { 2048, 1024, 1024, 1024, };
        cfg.ppo.critic.layerSizes = { 2048, 1024, 1024, 1024, };

	auto optim = ModelOptimType::ADAM;
	cfg.ppo.policy.optimType = optim;
	cfg.ppo.critic.optimType = optim;
	cfg.ppo.sharedHead.optimType = optim;

	auto activation = ModelActivationType::LEAKY_RELU;
	cfg.ppo.policy.activationType = activation;
	cfg.ppo.critic.activationType = activation;
	cfg.ppo.sharedHead.activationType = activation;

	bool addLayerNorm = true;
	cfg.ppo.policy.addLayerNorm = addLayerNorm;
	cfg.ppo.critic.addLayerNorm = addLayerNorm;
	cfg.ppo.sharedHead.addLayerNorm = addLayerNorm;

	cfg.sendMetrics = true; // Send metrics

	bool enableRender = false;
	cfg.renderMode = enableRender;
	cfg.ppo.deterministic = enableRender;

	// Make the learner with the environment creation function and the config we just made
	Learner* learner = new Learner(EnvCreateFunc, cfg, StepCallback);

	// Start learning!
	learner->Start();

	return EXIT_SUCCESS;
}
