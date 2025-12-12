#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t mlp_perf__32_32__1e_05_predict(const float *features, int32_t n_features);
int32_t mlp_perf__32_32__1e_05_regress(const float *features, int32_t n_features, float *out, int32_t out_length);
float mlp_perf__32_32__1e_05_regress1(const float *features, int32_t n_features);

#ifdef __cplusplus
}
#endif
