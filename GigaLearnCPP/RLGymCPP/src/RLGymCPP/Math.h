#pragma once
#include "Framework.h"

namespace RLGC {
	namespace Math {
		Vec RandVec(Vec min, Vec max);

		constexpr float VelToKPH(float vel) {
			return vel / (250.f / 9.f);
		}

		constexpr float KPHToVel(float vel) {
			return vel * (250.f / 9.f);
		}

		inline float cosine_similarity(const Vec& vec1, const Vec& vec2) {
			return vec1.Dot(vec2) / (vec1.Length() * vec2.Length());
		}

	}
}
