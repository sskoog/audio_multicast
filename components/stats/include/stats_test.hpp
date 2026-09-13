#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Executes the comprehensive verification self-test suite for the stats component.
 * Verifies mathematical precision, zero-allocation behavior, edge cases, and wraparound.
 * @return true if all test assertions pass, false otherwise.
 */
bool run_stats_self_test(void);

#ifdef __cplusplus
}
#endif
