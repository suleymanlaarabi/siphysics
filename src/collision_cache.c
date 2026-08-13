#include "collision_runtime.h"
#include <stdlib.h>
#include <string.h>

static inline void sip_contact_cache_key(
    ecs_entity_t entity_a,
    ecs_entity_t entity_b,
    float normal_x,
    float normal_y,
    ecs_entity_t *out_a,
    ecs_entity_t *out_b,
    float *out_normal_x,
    float *out_normal_y
) {
    if (entity_a < entity_b) {
        *out_a = entity_a;
        *out_b = entity_b;
        *out_normal_x = normal_x;
        *out_normal_y = normal_y;
    } else {
        *out_a = entity_b;
        *out_b = entity_a;
        *out_normal_x = -normal_x;
        *out_normal_y = -normal_y;
    }
}

void sip_contact_cache_reserve(
    SipCollisionRuntime *runtime,
    uint32_t needed
) {
    if (needed <= runtime->contact_cache_capacity) {
        return;
    }

    uint32_t capacity =
        runtime->contact_cache_capacity
            ? runtime->contact_cache_capacity
            : 64;

    while (capacity < needed) {
        capacity *= 2;
    }

    runtime->previous_contact_cache = realloc(
        runtime->previous_contact_cache,
        sizeof(*runtime->previous_contact_cache) * capacity
    );

    runtime->current_contact_cache = realloc(
        runtime->current_contact_cache,
        sizeof(*runtime->current_contact_cache) * capacity
    );

    runtime->contact_cache_scratch = realloc(
        runtime->contact_cache_scratch,
        sizeof(*runtime->contact_cache_scratch) * capacity
    );

    runtime->contact_cache_capacity = capacity;
    runtime->growth_count++;
}

void sip_contact_cache_begin_tick(
    SipCollisionRuntime *runtime
) {
    SipCachedContact *old_previous =
        runtime->previous_contact_cache;

    runtime->previous_contact_cache =
        runtime->current_contact_cache;

    runtime->current_contact_cache =
        old_previous;

    runtime->previous_contact_cache_count =
        runtime->current_contact_cache_count;

    runtime->current_contact_cache_count = 0;
    runtime->contact_cache_hit_count = 0;
}

static inline int sip_cached_contact_compare_key(
    const SipCachedContact *contact,
    ecs_entity_t a,
    ecs_entity_t b
) {
    if (contact->a < a) {
        return -1;
    }

    if (contact->a > a) {
        return 1;
    }

    if (contact->b < b) {
        return -1;
    }

    if (contact->b > b) {
        return 1;
    }

    return 0;
}

static const SipCachedContact *sip_contact_cache_find(
    const SipCollisionRuntime *runtime,
    ecs_entity_t a,
    ecs_entity_t b
) {
    uint32_t first = 0;
    uint32_t count =
        runtime->previous_contact_cache_count;

    while (count > 0) {
        const uint32_t step = count / 2;
        const uint32_t index = first + step;

        const SipCachedContact *cached =
            &runtime->previous_contact_cache[index];

        const int comparison =
            sip_cached_contact_compare_key(
                cached,
                a,
                b
            );

        if (comparison < 0) {
            first = index + 1;
            count -= step + 1;
        } else if (comparison > 0) {
            count = step;
        } else {
            return cached;
        }
    }

    return NULL;
}

void sip_contact_cache_restore(
    SipCollisionRuntime *runtime,
    SipSolverContact *contact,
    ecs_entity_t entity_a,
    ecs_entity_t entity_b
) {
    ecs_entity_t key_a;
    ecs_entity_t key_b;

    float canonical_normal_x;
    float canonical_normal_y;

    sip_contact_cache_key(
        entity_a,
        entity_b,
        contact->normal_x,
        contact->normal_y,
        &key_a,
        &key_b,
        &canonical_normal_x,
        &canonical_normal_y
    );

    const SipCachedContact *cached =
        sip_contact_cache_find(
            runtime,
            key_a,
            key_b
        );

    if (!cached) {
        return;
    }

    const float normal_dot =
        cached->normal_x * canonical_normal_x +
        cached->normal_y * canonical_normal_y;

    if (normal_dot < 0.95f) {
        return;
    }

    contact->normal_impulse =
        cached->normal_impulse;

    const float tangent_limit =
        contact->friction *
        contact->normal_impulse;

    contact->tangent_impulse =
        sip_clampf(
            cached->tangent_impulse,
            -tangent_limit,
            tangent_limit
        );

    runtime->contact_cache_hit_count++;
}

static void sip_contact_cache_sort(
    SipCollisionRuntime *runtime
) {
    const uint32_t count =
        runtime->current_contact_cache_count;

    if (count < 2) {
        return;
    }

    SipCachedContact *source =
        runtime->current_contact_cache;
    SipCachedContact *destination =
        runtime->contact_cache_scratch;

    for (uint32_t key = 0; key < 2; key++) {
        for (uint32_t shift = 0; shift < 64; shift += 8) {
            uint32_t counts[256] = {0};
            for (uint32_t i = 0; i < count; i++) {
                const ecs_entity_t value =
                    key == 0 ? source[i].b : source[i].a;
                counts[(uint32_t)((value >> shift) & 0xffu)]++;
            }

            uint32_t offsets[256];
            uint32_t offset = 0;
            for (uint32_t i = 0; i < 256; i++) {
                offsets[i] = offset;
                offset += counts[i];
            }

            for (uint32_t i = 0; i < count; i++) {
                const ecs_entity_t value =
                    key == 0 ? source[i].b : source[i].a;
                const uint32_t bucket =
                    (uint32_t)((value >> shift) & 0xffu);
                destination[offsets[bucket]++] = source[i];
            }

            SipCachedContact *temporary = source;
            source = destination;
            destination = temporary;
        }
    }

    if (source != runtime->current_contact_cache) {
        memcpy(
            runtime->current_contact_cache,
            source,
            sizeof(*source) * count
        );
    }
}

void sip_contact_cache_store(
    SipCollisionRuntime *runtime
) {
    sip_contact_cache_reserve(
        runtime,
        runtime->contact_count
    );

    runtime->current_contact_cache_count = 0;

    for (uint32_t i = 0;
         i < runtime->contact_count;
         i++) {
        const SipSolverContact *contact =
            &runtime->contacts[i];

        if (contact->sensor) {
            continue;
        }

        const SipBatchRef *batch_a =
            &runtime->batches[contact->batch_a];

        const SipBatchRef *batch_b =
            &runtime->batches[contact->batch_b];

        if (
            batch_a->body_type != SIP_BODY_DYNAMIC &&
            batch_b->body_type != SIP_BODY_DYNAMIC
        ) {
            continue;
        }

        const ecs_entity_t entity_a =
            batch_a->entities[contact->row_a];

        const ecs_entity_t entity_b =
            batch_b->entities[contact->row_b];

        SipCachedContact *cached =
            &runtime->current_contact_cache[
                runtime->current_contact_cache_count++
            ];

        sip_contact_cache_key(
            entity_a,
            entity_b,
            contact->normal_x,
            contact->normal_y,
            &cached->a,
            &cached->b,
            &cached->normal_x,
            &cached->normal_y
        );

        cached->normal_impulse =
            contact->normal_impulse;

        cached->tangent_impulse =
            contact->tangent_impulse;
    }

    sip_contact_cache_sort(runtime);
}
