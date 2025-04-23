
#include "math3d.h"
#include "stabilizer_types.h"
#include <math.h>
#include "controller_nn.h"
#include "log.h"
#include "param.h"
#include "usec_time.h"
#include "motors.h"


#define MAX_THRUST 0.15f
// PWM to thrust coefficients
#define A 2.130295e-11f
#define B 1.032633e-6f
#define C 5.484560e-4f

#define MAX_XY 1.0f
#define MAX_Z 1.0f
#define MAX_LIN_VEL_XY 3.0f
#define MAX_LIN_VEL_Z 1.0f

static bool enableBigQuad = false;

static float maxThrustFactor = 0.70f;
static bool relVel = true;
static bool relOmega = true;
static bool relXYZ = true;
static uint16_t freq = 240;

// static control_t_n control_n;
static control_t last_step_control_t;
static struct mat33 rot;
static float state_array[20];
// static float state_array[22];

static uint32_t usec_eval;

void controllerNNInit(void) {
	// last_step_control_n.thrust_0 = 0.0f;
	// last_step_control_n.thrust_1 = 0.0f;
	// last_step_control_n.thrust_2 = 0.0f;
	// last_step_control_n.thrust_3 = 0.0f;
}



bool controllerNNTest(void) {
	return true;
}


void controllerNNEnableBigQuad(void)
{
	enableBigQuad = true;
}

void controllerNN(control_t *control, 
				  const setpoint_t *setpoint, 
				  const sensorData_t *sensors, 
				  const state_t *state, 
				  const stabilizerStep_t stabilizerStep)
{
	control->controlMode = controlModeForce;
	if (!RATE_DO_EXECUTE(/*RATE_100_HZ*/freq, stabilizerStep)) {
		return;
	}


	// angular velocity
	float omega_roll = radians(sensors->gyro.x);
	float omega_pitch = radians(sensors->gyro.y);
	float omega_yaw = radians(sensors->gyro.z);

	struct quat q = mkquat(state->attitudeQuaternion.x, 
						   state->attitudeQuaternion.y, 
						   state->attitudeQuaternion.z, 
						   state->attitudeQuaternion.w);
	rot = quat2rotmat(q);


	struct vec setpoint_body = mvmul(mtranspose(rot), 
							   mkvec(state->position.x - setpoint->position.x, 
									 state->position.y - setpoint->position.y, 
									 state->position.z - setpoint->position.z));
	struct vec vel_body = mvmul(mtranspose(rot), 
							   mkvec(state->velocity.x, 
									 state->velocity.y, 
									 state->velocity.z));

	// the state vector
	
	// root_lin_vel_b
	state_array[0] = vel_body.x;
	state_array[1] = vel_body.y;
	state_array[2] = vel_body.z;

	// root_lin_vel_w
	state_array[3] = state->velocity.x;
	state_array[4] = state->velocity.y;
	state_array[5] = state->velocity.z;
	// root_ang_vel_b
	state_array[6] = omega_roll;
	state_array[7] = omega_pitch;
	state_array[8] = omega_yaw;
	// quat
	state_array[9] = state->attitudeQuaternion.x;
	state_array[10] = state->attitudeQuaternion.y;
	state_array[11] = state->attitudeQuaternion.z;
	state_array[12] = state->attitudeQuaternion.w;
	// window_setpoint
	state_array[13] = setpoint_body.x;
	state_array[14] = setpoint_body.y;
	state_array[15] = setpoint_body.z;
	// window_velocity
	state_array[16] = setpoint->velocity.x;
	state_array[17] = setpoint->velocity.y;
	state_array[18] = setpoint->velocity.z;
	// trajectory coeff
	state_array[19] = setpoint->traj_coeffs[0];
	state_array[20] = setpoint->traj_coeffs[1];
	state_array[21] = setpoint->traj_coeffs[2];
	state_array[22] = setpoint->traj_coeffs[3];
	state_array[23] = setpoint->traj_coeffs[4];
	state_array[24] = setpoint->traj_coeffs[5];
	state_array[25] = setpoint->traj_coeffs[6];
	
	

	// run the neural neural network
	uint64_t start = usecTimestamp();
	control->controlMode = controlModeForceTorque;

	networkEvaluate(control, state_array);
	usec_eval = (uint32_t) (usecTimestamp() - start);

	if (setpoint->mode.z == modeDisable) {
		control->normalizedForces[0] = 0.0f;
		control->normalizedForces[1] = 0.0f;
		control->normalizedForces[2] = 0.0f;
		control->normalizedForces[3] = 0.0f;
	}

	last_step_control_t = *control; // update last step_control


}

PARAM_GROUP_START(ctrlNN)
PARAM_ADD(PARAM_FLOAT, max_thrust, &maxThrustFactor)
PARAM_ADD(PARAM_UINT8, rel_vel, &relVel)
PARAM_ADD(PARAM_UINT8, rel_omega, &relOmega)
PARAM_ADD(PARAM_UINT8, rel_xyz, &relXYZ)
PARAM_ADD(PARAM_UINT16, freq, &freq)
PARAM_GROUP_STOP(ctrlNN)

LOG_GROUP_START(ctrlNN)
// LOG_ADD(LOG_FLOAT, out0, &control_n.thrust_0)
// LOG_ADD(LOG_FLOAT, out1, &control_n.thrust_1)
// LOG_ADD(LOG_FLOAT, out2, &control_n.thrust_2)
// LOG_ADD(LOG_FLOAT, out3, &control_n.thrust_3)

// LOG_ADD(LOG_FLOAT, in0, &state_array[0])
// LOG_ADD(LOG_FLOAT, in1, &state_array[1])
// LOG_ADD(LOG_FLOAT, in2, &state_array[2])

// LOG_ADD(LOG_FLOAT, in3, &state_array[3])
// LOG_ADD(LOG_FLOAT, in4, &state_array[4])
// LOG_ADD(LOG_FLOAT, in5, &state_array[5])

LOG_ADD(LOG_FLOAT, inm1, &state_array[16])
LOG_ADD(LOG_FLOAT, inm2, &state_array[17])
LOG_ADD(LOG_FLOAT, inm3, &state_array[18])
LOG_ADD(LOG_FLOAT, inm4, &state_array[19])
LOG_ADD(LOG_UINT32, usec_eval, &usec_eval)

LOG_GROUP_STOP(ctrlNN)