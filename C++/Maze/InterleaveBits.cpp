
#include "InterleaveBits.h"
#include <immintrin.h>

uint64_t interleaveBits(uint32_t x, uint32_t y)
{
	union uint128
	{
		__m128i combined_128b;
		uint64_t lower_64b;

		uint128(const __m128i value) { combined_128b = value; }
		uint128(const uint64_t value) { lower_64b = value; }

		operator __m128i() const { return combined_128b; }
	};

	uint128 x_128b(x), y_128b(y);

	return uint128(_mm_clmulepi64_si128(x_128b, x_128b, 0x00)).lower_64b
		+ 2 * uint128(_mm_clmulepi64_si128(y_128b, y_128b, 0x00)).lower_64b;
}
