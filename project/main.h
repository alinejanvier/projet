#ifndef MAIN_H
#define MAIN_H

#include "motors.h"
#include "sensors/proximity.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;


#ifdef __cplusplus
}
#endif

//turn the motors
void turn(double angle);
double incidence_angle(void);
void bouncing(double incidence_angle);



#endif
