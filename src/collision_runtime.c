#include "collision_runtime.h"
#include "siecs.h"
#include <stdlib.h>

#ifndef SIPHYSICS_BENCHMARK
enum {
    SIP_QUERY_DYNAMIC_CIRCLE,
    SIP_QUERY_KINEMATIC_CIRCLE,
    SIP_QUERY_STATIC_CIRCLE,
    SIP_QUERY_DYNAMIC_BOX,
    SIP_QUERY_KINEMATIC_BOX,
    SIP_QUERY_STATIC_BOX,
};
#endif

void sip_collision_runtime_destroy(void *ptr, uint32_t count) {
    (void)count;
    SipCollisionRuntime *runtime = ptr;
    free(runtime->proxies);
    free(runtime->proxy_scratch);
    free(runtime->box_geoms);
    free(runtime->batches);
    free(runtime->circle_circle_pairs);
    free(runtime->circle_box_pairs);
    free(runtime->box_box_pairs);
    free(runtime->contacts);
}

void sip_collision_runtime_reset(SipCollisionRuntime *runtime) {
    runtime->proxy_count = 0;
    runtime->batch_count = 0;
    runtime->circle_circle_count = 0;
    runtime->circle_box_count = 0;
    runtime->box_box_count = 0;
    runtime->contact_count = 0;
}

#ifndef SIPHYSICS_BENCHMARK
static ecs_query_id_t sip_circle_query(ecs_entity_t body, ecs_entity_t excluded_body_a,
                                       ecs_entity_t excluded_body_b, bool dynamic) {
    ecs_query_term_t terms[ECS_QUERY_TERM_CAPACITY] = {0};
    uint32_t field = 0;
    terms[field++] = dynamic ? ecs_inout(Position) : ecs_in(Position);
    if (body == ecs_id(Dynamic)) {
        terms[field++] = ecs_inout(Velocity);
        terms[field++] = ecs_in(InverseMass);
    } else if (body == ecs_id(Kinematic)) {
        terms[field++] = ecs_in(Velocity);
    }
    terms[field++] = ecs_in(CircleCollider);
    terms[field++] = ecs_in(CollisionMaterial);
    terms[field++] = ecs_in(CollisionFilter);
    terms[field++] = ecs_in_optional(Sensor);
    terms[field++] = (ecs_query_term_t){ body, EcsFilter };
    terms[field++] = (ecs_query_term_t){ excluded_body_a, EcsNot };
    terms[field++] = (ecs_query_term_t){ excluded_body_b, EcsNot };
    terms[field++] = (ecs_query_term_t){ ecs_id(BoxCollider), EcsNot };
    return ecs_query_init(&(ecs_query_desc_t){ .terms = { 
        terms[0], terms[1], terms[2], terms[3], terms[4], terms[5], terms[6], terms[7], terms[8], terms[9], terms[10]
    } });
}

static ecs_query_id_t sip_box_query(ecs_entity_t body, ecs_entity_t excluded_body_a,
                                    ecs_entity_t excluded_body_b, bool dynamic) {
    ecs_query_term_t terms[ECS_QUERY_TERM_CAPACITY] = {0};
    uint32_t field = 0;
    terms[field++] = dynamic ? ecs_inout(Position) : ecs_in(Position);
    if (body == ecs_id(Dynamic)) {
        terms[field++] = ecs_inout(Velocity);
        terms[field++] = ecs_in(InverseMass);
    } else if (body == ecs_id(Kinematic)) {
        terms[field++] = ecs_in(Velocity);
    }
    terms[field++] = ecs_in(Rotation);
    terms[field++] = ecs_in(BoxCollider);
    terms[field++] = ecs_in(CollisionMaterial);
    terms[field++] = ecs_in(CollisionFilter);
    terms[field++] = ecs_in_optional(Sensor);
    terms[field++] = (ecs_query_term_t){ body, EcsFilter };
    terms[field++] = (ecs_query_term_t){ excluded_body_a, EcsNot };
    terms[field++] = (ecs_query_term_t){ excluded_body_b, EcsNot };
    terms[field++] = (ecs_query_term_t){ ecs_id(CircleCollider), EcsNot };
    return ecs_query_init(&(ecs_query_desc_t){ .terms = {
        terms[0], terms[1], terms[2], terms[3], terms[4], terms[5], terms[6], terms[7], terms[8], terms[9], terms[10], terms[11]
    } });
}

void siphysics_collision_init(void) {
    SipCollisionRuntime *runtime = ecs_get_resource(SipCollisionRuntime);
    runtime->queries[SIP_QUERY_DYNAMIC_CIRCLE] =
        sip_circle_query(ecs_id(Dynamic), ecs_id(Kinematic), ecs_id(Static), true);
    runtime->queries[SIP_QUERY_KINEMATIC_CIRCLE] =
        sip_circle_query(ecs_id(Kinematic), ecs_id(Dynamic), ecs_id(Static), false);
    runtime->queries[SIP_QUERY_STATIC_CIRCLE] =
        sip_circle_query(ecs_id(Static), ecs_id(Dynamic), ecs_id(Kinematic), false);
    runtime->queries[SIP_QUERY_DYNAMIC_BOX] =
        sip_box_query(ecs_id(Dynamic), ecs_id(Kinematic), ecs_id(Static), true);
    runtime->queries[SIP_QUERY_KINEMATIC_BOX] =
        sip_box_query(ecs_id(Kinematic), ecs_id(Dynamic), ecs_id(Static), false);
    runtime->queries[SIP_QUERY_STATIC_BOX] =
        sip_box_query(ecs_id(Static), ecs_id(Dynamic), ecs_id(Kinematic), false);
}

void siphysics_collision_register_system(ecs_system_id_t integrate_velocity,
                                         ecs_system_id_t integrate_angular_velocity) {
    ecs_system(
        {
            .name = "CollisionStep",
            .phase = siphysics_phase,
            .after = { integrate_velocity, integrate_angular_velocity },
            .callback = siphysics_collision_step,
        }
    );
}
#endif
