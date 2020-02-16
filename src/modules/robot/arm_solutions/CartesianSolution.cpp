#include "CartesianSolution.h"
#include "ActuatorCoordinates.h"
#include <math.h>

#define angle (90.961*(M_PI/180.0))
void CartesianSolution::cartesian_to_actuator( const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const {
    actuator_mm[ALPHA_STEPPER] = cartesian_mm[X_AXIS] - cartesian_mm[Y_AXIS] * (1 / tan(angle));
    actuator_mm[BETA_STEPPER ] = cartesian_mm[Y_AXIS] * (1 / sin(angle));
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
}

void CartesianSolution::actuator_to_cartesian( const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const {
    cartesian_mm[ALPHA_STEPPER] = actuator_mm[X_AXIS] + actuator_mm[Y_AXIS] * cos(angle);
    cartesian_mm[BETA_STEPPER ] = actuator_mm[Y_AXIS] * sin(angle);
    cartesian_mm[GAMMA_STEPPER] = actuator_mm[Z_AXIS];
}
