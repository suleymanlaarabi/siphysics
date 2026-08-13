#ifndef SIPHYSICS_INTERNAL_H
#define SIPHYSICS_INTERNAL_H

#include "collision_runtime.h"

ECS_RESOURCE_DECLARE(SipPhysicsRuntime, {
    double accumulator;
    uint64_t tick;

    ecs_query_id_t gravity_query;
    ecs_query_id_t velocity_query;
    ecs_query_id_t angular_velocity_query;
});

void siphysics_register_systems(void);

#endif
