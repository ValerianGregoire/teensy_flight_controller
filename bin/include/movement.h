#ifndef MOVEMENT
#define MOVEMENT

#include "common.h"

/*
Navigation task. Reads data through SPI and sends it to optical_flow_data_new.
Sets a flag in all_updated_reg when called.
*/
void navigation(void *parameters);

/*
Mapping task. Reads data from lidar_data_old, and updates map.
*/
void mapping(void *parameters);

/*
Wire task. Reads data from map, and updates wire_position.
*/
void wire(void *parameters);

#endif