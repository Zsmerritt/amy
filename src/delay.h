#ifndef _DELAY_H

// How many bits used for fractional part of delay line index.
#define DELAY_INDEX_FRAC_BITS 15
// The number of bits used to hold the delay line index.
#define DELAY_INDEX_BITS (31 - DELAY_INDEX_FRAC_BITS)

#include "amy.h"

delay_line_t *new_delay_line(int len, int fixed_delay, int ram_type /* e.g. MALLOC_CAP_INTERNAL */);
void free_delay_line(delay_line_t *d);

void apply_variable_delay(SAMPLE *block, delay_line_t *delay_line, SAMPLE *delay_samples, SAMPLE mod_scale, SAMPLE mix_level, SAMPLE feedback_level);
void apply_fixed_delay(SAMPLE *block, delay_line_t *delay_line, SAMPLE mix_level, SAMPLE feedback, SAMPLE filter_coef);

reverb_params_t *new_reverb();
void delete_reverb(reverb_params_t *rev);
void config_stereo_reverb(reverb_params_t *rev, float a_liveness, float crossover_hz, float damping);
bool init_stereo_reverb(reverb_params_t *rev);
void deinit_stereo_reverb(reverb_params_t *rev);
void stereo_reverb(reverb_params_t *rev, SAMPLE *r_in, SAMPLE *l_in, SAMPLE *r_out, SAMPLE *l_out, int n_samples, SAMPLE level);
// Aux-return variant: ACCUMULATES level*wet into {r,l}_acc, never the dry.
void stereo_reverb_wet(reverb_params_t *rev, SAMPLE *r_in, SAMPLE *l_in, SAMPLE *r_acc, SAMPLE *l_acc, int n_samples, SAMPLE level);

#endif // !_DELAY_H
