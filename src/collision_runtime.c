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
    free(runtime->previous_contact_cache);
    free(runtime->current_contact_cache);
    free(runtime->contact_cache_scratch);
    free(runtime->previous_pairs);
    free(runtime->current_pairs);
    free(runtime->event_pair_scratch);
}

void sip_collision_runtime_reset(SipCollisionRuntime *runtime) {
    sip_contact_cache_begin_tick(runtime);

    SipEventPair *event_pairs = runtime->previous_pairs;
    runtime->previous_pairs = runtime->current_pairs;
    runtime->current_pairs = event_pairs;
    runtime->previous_event_pair_count = runtime->current_event_pair_count;
    runtime->current_event_pair_count = 0;
    runtime->proxy_count = 0;
    runtime->batch_count = 0;
    runtime->circle_circle_count = 0;
    runtime->circle_box_count = 0;
    runtime->box_box_count = 0;
    runtime->contact_count = 0;
    runtime->event_dispatch_count = 0;
}

static void sip_reserve(
    void **memory,
    uint32_t *capacity,
    uint32_t needed,
    size_t element_size,
    SipCollisionRuntime *runtime
) {
    if (needed <= *capacity) {
        return;
    }
    *memory = realloc(*memory, element_size * needed);
    *capacity = needed;
    runtime->growth_count++;
}

void sip_collision_runtime_reserve(
    SipCollisionRuntime *runtime,
    const SipCollisionCapacity *capacity
) {
    if (capacity->proxy_capacity > runtime->proxy_capacity) {
        runtime->proxies =
            realloc(runtime->proxies, sizeof(*runtime->proxies) * capacity->proxy_capacity);
        runtime->proxy_scratch = realloc(
            runtime->proxy_scratch,
            sizeof(*runtime->proxy_scratch) * capacity->proxy_capacity
        );
        runtime->box_geoms =
            realloc(runtime->box_geoms, sizeof(*runtime->box_geoms) * capacity->proxy_capacity);
        runtime->proxy_capacity = capacity->proxy_capacity;
        runtime->growth_count++;
    }

    sip_reserve(
        (void **)&runtime->circle_circle_pairs,
        &runtime->circle_circle_capacity,
        capacity->pair_capacity,
        sizeof(*runtime->circle_circle_pairs),
        runtime
    );
    sip_reserve(
        (void **)&runtime->circle_box_pairs,
        &runtime->circle_box_capacity,
        capacity->pair_capacity,
        sizeof(*runtime->circle_box_pairs),
        runtime
    );
    sip_reserve(
        (void **)&runtime->box_box_pairs,
        &runtime->box_box_capacity,
        capacity->pair_capacity,
        sizeof(*runtime->box_box_pairs),
        runtime
    );
    sip_reserve(
        (void **)&runtime->contacts,
        &runtime->contact_capacity,
        capacity->contact_capacity,
        sizeof(*runtime->contacts),
        runtime
    );
    sip_contact_cache_reserve(runtime, capacity->contact_capacity);
    if (capacity->event_pair_capacity > runtime->event_pair_capacity) {
        runtime->previous_pairs = realloc(
            runtime->previous_pairs,
            sizeof(*runtime->previous_pairs) * capacity->event_pair_capacity
        );
        runtime->current_pairs = realloc(
            runtime->current_pairs,
            sizeof(*runtime->current_pairs) * capacity->event_pair_capacity
        );
        runtime->event_pair_scratch = realloc(
            runtime->event_pair_scratch,
            sizeof(*runtime->event_pair_scratch) * capacity->event_pair_capacity
        );
        runtime->event_pair_capacity = capacity->event_pair_capacity;
        runtime->growth_count++;
    }
}

#ifndef SIPHYSICS_BENCHMARK
static ecs_query_id_t sip_query_init_terms(
    const ecs_component_term_t *terms,
    uint32_t count
) {
    ecs_query_desc_t desc = {0};
    for (uint32_t i = 0; i < count; i++) {
        desc.components[i] = terms[i];
    }
    return ecs_query_init(&desc);
}

static ecs_query_id_t sip_circle_query(
    ecs_entity_t body,
    ecs_entity_t excluded_body_a,
    ecs_entity_t excluded_body_b,
    bool dynamic,
    bool sensor,
    bool events
) {
    ecs_component_term_t terms[ECS_QUERY_TERM_CAPACITY] = { 0 };
    uint32_t field = 0;
    terms[field++] = dynamic ? ecs_inout(Position) : ecs_in(Position);
    if (body == ecs_id(Dynamic)) {
        terms[field++] = ecs_inout(Velocity);
        terms[field++] = ecs_inout(AngularVelocity);
        terms[field++] = ecs_in(InverseMass);
        terms[field++] = ecs_inout(InverseInertia);
    } else if (body == ecs_id(Kinematic)) {
        terms[field++] = ecs_in(Velocity);
        terms[field++] = ecs_in(AngularVelocity);
    }
    terms[field++] = ecs_in(CircleCollider);
    terms[field++] = ecs_in(CollisionMaterial);
    terms[field++] = ecs_in(CollisionFilter);
    terms[field++] = sensor ? ecs_filter(Sensor) : ecs_not(Sensor);
    terms[field++] = events ? ecs_filter(CollisionEvents) : ecs_not(CollisionEvents);
    terms[field++] = (ecs_component_term_t){ body, EcsFilter };
    terms[field++] = (ecs_component_term_t){ excluded_body_a, EcsNot };
    terms[field++] = (ecs_component_term_t){ excluded_body_b, EcsNot };
    terms[field++] = (ecs_component_term_t){ ecs_id(BoxCollider), EcsNot };
    return sip_query_init_terms(terms, field);
}

static ecs_query_id_t sip_box_query(
    ecs_entity_t body,
    ecs_entity_t excluded_body_a,
    ecs_entity_t excluded_body_b,
    bool dynamic,
    bool sensor,
    bool events
) {
    ecs_component_term_t terms[ECS_QUERY_TERM_CAPACITY] = { 0 };
    uint32_t field = 0;
    terms[field++] = dynamic ? ecs_inout(Position) : ecs_in(Position);
    if (body == ecs_id(Dynamic)) {
        terms[field++] = ecs_inout(Velocity);
        terms[field++] = ecs_inout(AngularVelocity);
        terms[field++] = ecs_in(InverseMass);
        terms[field++] = ecs_inout(InverseInertia);
    } else if (body == ecs_id(Kinematic)) {
        terms[field++] = ecs_in(Velocity);
        terms[field++] = ecs_in(AngularVelocity);
    }
    terms[field++] = ecs_in(Rotation);
    terms[field++] = ecs_in(BoxCollider);
    terms[field++] = ecs_in(CollisionMaterial);
    terms[field++] = ecs_in(CollisionFilter);
    terms[field++] = sensor ? ecs_filter(Sensor) : ecs_not(Sensor);
    terms[field++] = events ? ecs_filter(CollisionEvents) : ecs_not(CollisionEvents);
    terms[field++] = (ecs_component_term_t){ body, EcsFilter };
    terms[field++] = (ecs_component_term_t){ excluded_body_a, EcsNot };
    terms[field++] = (ecs_component_term_t){ excluded_body_b, EcsNot };
    terms[field++] = (ecs_component_term_t){ ecs_id(CircleCollider), EcsNot };
    return sip_query_init_terms(terms, field);
}

void siphysics_collision_init(void) {
    SipCollisionRuntime *runtime = ecs_get_resource(SipCollisionRuntime);
    for (uint32_t flags = 0; flags < 4; flags++) {
        const bool sensor = (flags & 2u) != 0;
        const bool events = (flags & 1u) != 0;
        runtime->queries[SIP_QUERY_DYNAMIC_CIRCLE * 4 + flags] = sip_circle_query(
            ecs_id(Dynamic),
            ecs_id(Kinematic),
            ecs_id(Static),
            true,
            sensor,
            events
        );
        runtime->queries[SIP_QUERY_KINEMATIC_CIRCLE * 4 + flags] = sip_circle_query(
            ecs_id(Kinematic),
            ecs_id(Dynamic),
            ecs_id(Static),
            false,
            sensor,
            events
        );
        runtime->queries[SIP_QUERY_STATIC_CIRCLE * 4 + flags] = sip_circle_query(
            ecs_id(Static),
            ecs_id(Dynamic),
            ecs_id(Kinematic),
            false,
            sensor,
            events
        );
        runtime->queries[SIP_QUERY_DYNAMIC_BOX * 4 + flags] =
            sip_box_query(ecs_id(Dynamic), ecs_id(Kinematic), ecs_id(Static), true, sensor, events);
        runtime->queries[SIP_QUERY_KINEMATIC_BOX * 4 + flags] = sip_box_query(
            ecs_id(Kinematic),
            ecs_id(Dynamic),
            ecs_id(Static),
            false,
            sensor,
            events
        );
        runtime->queries[SIP_QUERY_STATIC_BOX * 4 + flags] = sip_box_query(
            ecs_id(Static),
            ecs_id(Dynamic),
            ecs_id(Kinematic),
            false,
            sensor,
            events
        );
    }
}

#endif
