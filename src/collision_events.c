#include "collision_runtime.h"
#include <string.h>

SIPHYSICS_API ecs_event_t SipCollisionEnter = UINT16_MAX;
SIPHYSICS_API ecs_event_t SipCollisionStay = UINT16_MAX;
SIPHYSICS_API ecs_event_t SipCollisionExit = UINT16_MAX;

static int sip_event_pair_compare(const SipEventPair *a, const SipEventPair *b) {
    if (a->a != b->a) {
        return a->a < b->a ? -1 : 1;
    }
    return a->b != b->b ? (a->b < b->b ? -1 : 1) : 0;
}

static void sip_event_pair_sort(SipEventPair *pairs, uint32_t count,
                                SipEventPair *scratch) {
    if (count < 2) {
        return;
    }

    SipEventPair *source = pairs;
    SipEventPair *destination = scratch;
    for (uint32_t key = 0; key < 2; key++) {
        for (uint32_t shift = 0; shift < 64; shift += 8) {
            uint32_t counts[256] = {0};
            for (uint32_t i = 0; i < count; i++) {
                const ecs_entity_t value = key == 0 ? source[i].b : source[i].a;
                counts[(uint32_t)((value >> shift) & 0xffu)]++;
            }
            uint32_t offsets[256];
            uint32_t offset = 0;
            for (uint32_t i = 0; i < 256; i++) {
                offsets[i] = offset;
                offset += counts[i];
            }
            for (uint32_t i = 0; i < count; i++) {
                const ecs_entity_t value = key == 0 ? source[i].b : source[i].a;
                const uint32_t bucket = (uint32_t)((value >> shift) & 0xffu);
                destination[offsets[bucket]++] = source[i];
            }
            SipEventPair *temporary = source;
            source = destination;
            destination = temporary;
        }
    }
    if (source != pairs) {
        memcpy(pairs, source, sizeof(*pairs) * count);
    }
}

static void sip_trigger_collision_side(const SipEventPair *pair, ecs_event_t event,
                                       bool side_b) {
    const ecs_entity_t self = side_b ? pair->b : pair->a;
    const ecs_entity_t other = side_b ? pair->a : pair->b;
    if (!(side_b ? pair->interest_b : pair->interest_a) || !ecs_is_alive(self)) {
        return;
    }
    SipCollisionEvent payload = {
        .self = self,
        .other = other,
        .normal_x = side_b ? -pair->normal_x : pair->normal_x,
        .normal_y = side_b ? -pair->normal_y : pair->normal_y,
        .point_x = pair->point_x,
        .point_y = pair->point_y,
        .penetration = pair->penetration,
    };
    ecs_observer_trigger(self, event, &payload);
}

static void sip_dispatch_pair(SipCollisionRuntime *runtime, const SipEventPair *pair,
                              ecs_event_t event) {
    if (pair->interest_a && ecs_is_alive(pair->a)) {
        sip_trigger_collision_side(pair, event, false);
        runtime->event_dispatch_count++;
    }
    if (pair->interest_b && ecs_is_alive(pair->b)) {
        sip_trigger_collision_side(pair, event, true);
        runtime->event_dispatch_count++;
    }
}

void sip_collision_event_diff(SipCollisionRuntime *runtime) {
    sip_event_pair_sort(runtime->current_pairs, runtime->current_event_pair_count,
                        runtime->event_pair_scratch);

    uint32_t previous = 0;
    uint32_t current = 0;
    while (current < runtime->current_event_pair_count ||
           previous < runtime->previous_event_pair_count) {
        if (previous == runtime->previous_event_pair_count) {
            sip_dispatch_pair(runtime, &runtime->current_pairs[current++],
                              SipCollisionEnter);
            continue;
        }
        if (current == runtime->current_event_pair_count) {
            sip_dispatch_pair(runtime, &runtime->previous_pairs[previous++],
                              SipCollisionExit);
            continue;
        }

        const SipEventPair *old_pair = &runtime->previous_pairs[previous];
        const SipEventPair *new_pair = &runtime->current_pairs[current];
        const int comparison = sip_event_pair_compare(old_pair, new_pair);
        if (comparison < 0) {
            sip_dispatch_pair(runtime, old_pair, SipCollisionExit);
            previous++;
        } else if (comparison > 0) {
            sip_dispatch_pair(runtime, new_pair, SipCollisionEnter);
            current++;
        } else {
            sip_dispatch_pair(runtime, new_pair, SipCollisionStay);
            previous++;
            current++;
        }
    }
}
