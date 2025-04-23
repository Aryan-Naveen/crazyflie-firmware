#ifndef __NETWORK_EVALUATE_H__
#define __NETWORK_EVALUATE_H__

#include <math.h>
#include "stabilizer_types.h"

const float T2W = 1.9;
const float RW = 0.01;
const float MS = 0.2766;

/*
 * since the network outputs thrust on each motor,
 * we need to define a struct which stores the values
*/
void networkEvaluate(control_t *control, const float *state_array);

#endif