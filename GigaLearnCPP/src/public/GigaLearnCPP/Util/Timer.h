#pragma once
#include "../Framework.h"

namespace GGL {
	struct Timer {
		std::chrono::steady_clock::time_point startTime;

		Timer() {
			Reset();
		}

		// Returns elapsed time in seconds
		double Elapsed() {
			auto endTime = std::chrono::steady_clock::now(); // <-- utiliser steady_clock
			std::chrono::duration<double> elapsed = endTime - startTime;
			return elapsed.count();
		}

		void Reset() {
			startTime = std::chrono::steady_clock::now(); // <-- utiliser steady_clock
		}
	};
}
