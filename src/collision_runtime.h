#ifndef SIPHYSICS_COLLISION_RUNTIME_H
#define SIPHYSICS_COLLISION_RUNTIME_H

#include "collision_internal.h"
#include "siphysics/collision.h"
#include "siphysics/components.h"
#include "siphysics/physics.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum SipShape {
    SIP_SHAPE_CIRCLE = 1,
    SIP_SHAPE_BOX = 2,
} SipShape;

typedef enum SipBodyType {
    SIP_BODY_DYNAMIC = 1,
    SIP_BODY_KINEMATIC = 2,
    SIP_BODY_STATIC = 3,
} SipBodyType;

typedef struct SipBoxGeom {
    float axis_x_x;
    float axis_x_y;
    float axis_y_x;
    float axis_y_y;
} SipBoxGeom;

typedef struct SipBatchRef {
    Position *positions;
    Velocity *velocities;
    InverseMass *inverse_masses;
    const Rotation *rotations;
    const CircleCollider *circles;
    const BoxCollider *boxes;
    const CollisionMaterial *materials;
    const CollisionFilter *filters;
    ecs_entity_t *entities;
    uint32_t count;
    uint8_t shape;
    uint8_t body_type;
    uint8_t sensor;
    uint8_t event_enabled;
} SipBatchRef;

typedef struct SipProxy {
    float min_x;
    float max_x;
    float min_y;
    float max_y;

    uint32_t batch_index;
    uint32_t row;
    uint32_t box_geom_index;

    uint32_t layer;
    uint32_t mask;

    uint8_t shape;
    uint8_t body_type;
    uint8_t sensor;
    uint8_t event_interest;
} SipProxy;

typedef struct SipPair {
    uint32_t proxy_a;
    uint32_t proxy_b;
} SipPair;

typedef struct SipSolverContact {
    uint32_t batch_a;
    uint32_t row_a;
    uint32_t batch_b;
    uint32_t row_b;

    float normal_x;
    float normal_y;
    float penetration;

    float restitution;
    float friction;

    float normal_impulse;
    float tangent_impulse;

    uint8_t body_type_a;
    uint8_t body_type_b;
    uint8_t sensor;
} SipSolverContact;

_Static_assert(sizeof(SipSolverContact) <= 64,
               "SipSolverContact must remain compact for solver bandwidth");

typedef struct SipEventPair {
    ecs_entity_t a;
    ecs_entity_t b;

    float normal_x;
    float normal_y;
    float point_x;
    float point_y;
    float penetration;

    uint8_t interest_a;
    uint8_t interest_b;
} SipEventPair;

ECS_RESOURCE_DECLARE(SipCollisionRuntime, {
    SipProxy *proxies;
    uint32_t proxy_count;
    uint32_t proxy_capacity;

    SipProxy *proxy_scratch;
    SipBoxGeom *box_geoms;

    SipBatchRef *batches;
    uint32_t batch_count;
    uint32_t batch_capacity;

    SipPair *circle_circle_pairs;
    uint32_t circle_circle_count;
    uint32_t circle_circle_capacity;
    SipPair *circle_box_pairs;
    uint32_t circle_box_count;
    uint32_t circle_box_capacity;
    SipPair *box_box_pairs;
    uint32_t box_box_count;
    uint32_t box_box_capacity;

    SipSolverContact *contacts;
    uint32_t contact_count;
    uint32_t contact_capacity;

    SipEventPair *previous_pairs;
    uint32_t previous_event_pair_count;
    SipEventPair *current_pairs;
    uint32_t current_event_pair_count;
    SipEventPair *event_pair_scratch;
    uint32_t event_pair_capacity;

    ecs_query_id_t queries[24];
    uint32_t event_dispatch_count;
    uint64_t growth_count;
});

void siphysics_collision_init(void);
void siphysics_collision_register_system(ecs_system_id_t integrate_velocity,
                                         ecs_system_id_t integrate_angular_velocity);
void siphysics_collision_step(ecs_iter_t *it);

void sip_collision_runtime_reset(SipCollisionRuntime *runtime);
void sip_collision_runtime_destroy(void *ptr, uint32_t count);
void sip_broadphase(SipCollisionRuntime *runtime);
void sip_radix_sort(SipCollisionRuntime *runtime);
void sip_generate_pairs(SipCollisionRuntime *runtime);
void sip_collision_collect(SipCollisionRuntime *runtime);
void sip_collision_narrowphase(SipCollisionRuntime *runtime);
void sip_collision_solve(SipCollisionRuntime *runtime, const SipSettings *settings);
void sip_collision_runtime_reserve(SipCollisionRuntime *runtime,
                                   const SipCollisionCapacity *capacity);
void sip_collision_event_diff(SipCollisionRuntime *runtime);

#endif
