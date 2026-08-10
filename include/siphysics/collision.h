#ifndef SIPHYSICS_COLLISION_H
#define SIPHYSICS_COLLISION_H

#include "siphysics/bake_config.h"
#include <siecs.h>

#ifdef __cplusplus
extern "C" {
#endif

SIPHYSICS_API extern ecs_event_t SipCollisionEnter;
SIPHYSICS_API extern ecs_event_t SipCollisionStay;
SIPHYSICS_API extern ecs_event_t SipCollisionExit;

typedef struct SipCollisionEvent {
    ecs_entity_t self;
    ecs_entity_t other;

    float normal_x;
    float normal_y;

    float point_x;
    float point_y;

    float penetration;
} SipCollisionEvent;

#ifdef __cplusplus
}
#endif

#endif
