#ifndef RF_MODEL_H
#define RF_MODEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Random Forest classifier exported from emlearn.
    //
    // The model expects a fixed-length int16_t feature vector.
    // Check rf_model.c: find the highest index used (e.g. features[405])
    // and use that +1 as FEATURES_RF_DIM.
    int32_t rf_perf_25_6_predict(const int16_t *features, int32_t features_length);

#ifdef __cplusplus
}
#endif

#endif // RF_MODEL_H
