


    // !!! This file is generated using emlearn !!!

    #include <stdint.h>
    

static inline int32_t rf_wt_2_6_tree_0(const int16_t *features, int32_t features_length) {
          if (features[405] < -3571) {
              if (features[394] < -8321) {
                  return 2;
              } else {
                  return 1;
              }
          } else {
              if (features[213] < -2064) {
                  if (features[250] < 4215) {
                      return 0;
                  } else {
                      return 4;
                  }
              } else {
                  if (features[328] < -2965) {
                      if (features[234] < -7599) {
                          if (features[350] < -4340) {
                              return 2;
                          } else {
                              return 3;
                          }
                      } else {
                          if (features[124] < 3229) {
                              return 2;
                          } else {
                              return 3;
                          }
                      }
                  } else {
                      if (features[231] < 2814) {
                          if (features[279] < -3289) {
                              return 0;
                          } else {
                              if (features[431] < 4031) {
                                  return 4;
                              } else {
                                  return 5;
                              }
                          }
                      } else {
                          return 5;
                      }
                  }
              }
          }
        }
        

static inline int32_t rf_wt_2_6_tree_1(const int16_t *features, int32_t features_length) {
          if (features[267] < 4113) {
              if (features[243] < -2048) {
                  if (features[186] < -3331) {
                      if (features[395] < 386) {
                          return 0;
                      } else {
                          return 2;
                      }
                  } else {
                      if (features[483] < -323) {
                          return 1;
                      } else {
                          if (features[380] < -3566) {
                              return 4;
                          } else {
                              return 0;
                          }
                      }
                  }
              } else {
                  if (features[444] < -1429) {
                      if (features[138] < -310) {
                          if (features[592] < -4958) {
                              return 2;
                          } else {
                              if (features[59] < 435) {
                                  return 0;
                              } else {
                                  return 3;
                              }
                          }
                      } else {
                          if (features[504] < -1250) {
                              return 4;
                          } else {
                              return 0;
                          }
                      }
                  } else {
                      if (features[54] < 157) {
                          if (features[196] < -316) {
                              return 2;
                          } else {
                              if (features[10] < 474) {
                                  return 0;
                              } else {
                                  return 1;
                              }
                          }
                      } else {
                          if (features[166] < 3145) {
                              if (features[381] < 1205) {
                                  return 4;
                              } else {
                                  return 0;
                              }
                          } else {
                              if (features[310] < 2036) {
                                  return 3;
                              } else {
                                  return 4;
                              }
                          }
                      }
                  }
              }
          } else {
              return 5;
          }
        }
        

int32_t rf_wt_2_6_predict(const int16_t *features, int32_t features_length) {

        int32_t votes[6] = {0,};
        int32_t _class = -1;

        _class = rf_wt_2_6_tree_0(features, features_length); votes[_class] += 1;
    _class = rf_wt_2_6_tree_1(features, features_length); votes[_class] += 1;
    
        int32_t most_voted_class = -1;
        int32_t most_voted_votes = 0;
        for (int32_t i=0; i<6; i++) {

            if (votes[i] > most_voted_votes) {
                most_voted_class = i;
                most_voted_votes = votes[i];
            }
        }
        return most_voted_class;
    }
    

int rf_wt_2_6_predict_proba(const int16_t *features, int32_t features_length, float *out, int out_length) {

        int32_t _class = -1;

        for (int i=0; i<out_length; i++) {
            out[i] = 0.0f;
        }

        _class = rf_wt_2_6_tree_0(features, features_length); out[_class] += 1.0f;
    _class = rf_wt_2_6_tree_1(features, features_length); out[_class] += 1.0f;
    
        // compute mean
        for (int i=0; i<out_length; i++) {
            out[i] = out[i] / 2;
        }
        return 0;
    }
    