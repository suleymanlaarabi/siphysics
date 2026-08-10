#include "collision_runtime.h"
#include <stdlib.h>
#include <string.h>

static uint32_t sip_float_key(float value) {
    union {
        float value;
        uint32_t bits;
    } converted = { .value = value };
    const uint32_t sign = converted.bits >> 31;
    return converted.bits ^ ((0u - sign) | 0x80000000u);
}

void sip_radix_sort(SipCollisionRuntime *runtime) {
    SipProxy *source = runtime->proxies;
    SipProxy *destination = runtime->proxy_scratch;
    for (uint32_t shift = 0; shift < 32; shift += 8) {
        uint32_t counts[256] = {0};
        for (uint32_t i = 0; i < runtime->proxy_count; i++) {
            counts[(sip_float_key(source[i].min_x) >> shift) & 0xffu]++;
        }
        uint32_t offsets[256];
        uint32_t offset = 0;
        for (uint32_t i = 0; i < 256; i++) {
            offsets[i] = offset;
            offset += counts[i];
        }
        for (uint32_t i = 0; i < runtime->proxy_count; i++) {
            const uint32_t bucket = (sip_float_key(source[i].min_x) >> shift) & 0xffu;
            destination[offsets[bucket]++] = source[i];
        }
        SipProxy *temporary = source;
        source = destination;
        destination = temporary;
    }
    if (source != runtime->proxies) {
        memcpy(runtime->proxies, source, sizeof(*source) * runtime->proxy_count);
    }
}

static void sip_reserve_pairs(SipPair **pairs, uint32_t *capacity,
                              uint32_t needed, SipCollisionRuntime *runtime) {
    if (needed <= *capacity) {
        return;
    }
    uint32_t new_capacity = *capacity ? *capacity : 64;
    while (new_capacity < needed) {
        new_capacity *= 2;
    }
    *pairs = realloc(*pairs, sizeof(**pairs) * new_capacity);
    *capacity = new_capacity;
    runtime->growth_count++;
}

static void sip_push_pair(SipCollisionRuntime *runtime, uint32_t proxy_a, uint32_t proxy_b) {
    SipProxy *a = &runtime->proxies[proxy_a];
    SipProxy *b = &runtime->proxies[proxy_b];
    if (a->shape == SIP_SHAPE_BOX && b->shape == SIP_SHAPE_CIRCLE) {
        const uint32_t proxy = proxy_a;
        proxy_a = proxy_b;
        proxy_b = proxy;
    }
    a = &runtime->proxies[proxy_a];
    b = &runtime->proxies[proxy_b];
    if (a->shape == b->shape && a->body_type != SIP_BODY_DYNAMIC &&
        b->body_type == SIP_BODY_DYNAMIC) {
        const uint32_t proxy = proxy_a;
        proxy_a = proxy_b;
        proxy_b = proxy;
    }

    a = &runtime->proxies[proxy_a];
    b = &runtime->proxies[proxy_b];
    SipPair **pairs;
    uint32_t *count;
    uint32_t *capacity;
    if (a->shape == SIP_SHAPE_CIRCLE && b->shape == SIP_SHAPE_CIRCLE) {
        pairs = &runtime->circle_circle_pairs;
        count = &runtime->circle_circle_count;
        capacity = &runtime->circle_circle_capacity;
    } else if (a->shape == SIP_SHAPE_CIRCLE || b->shape == SIP_SHAPE_CIRCLE) {
        pairs = &runtime->circle_box_pairs;
        count = &runtime->circle_box_count;
        capacity = &runtime->circle_box_capacity;
    } else {
        pairs = &runtime->box_box_pairs;
        count = &runtime->box_box_count;
        capacity = &runtime->box_box_capacity;
    }
    sip_reserve_pairs(pairs, capacity, *count + 1, runtime);
    (*pairs)[(*count)++] = (SipPair){ .proxy_a = proxy_a, .proxy_b = proxy_b };
}

void sip_generate_pairs(SipCollisionRuntime *runtime) {
    runtime->circle_circle_count = 0;
    runtime->circle_box_count = 0;
    runtime->box_box_count = 0;
    for (uint32_t i = 0; i < runtime->proxy_count; i++) {
        const SipProxy *a = &runtime->proxies[i];
        for (uint32_t j = i + 1; j < runtime->proxy_count; j++) {
            const SipProxy *b = &runtime->proxies[j];
            if (b->min_x > a->max_x) {
                break;
            }
            if (b->max_y < a->min_y || b->min_y > a->max_y ||
                (a->body_type == SIP_BODY_STATIC && b->body_type == SIP_BODY_STATIC) ||
                !(a->layer & b->mask) || !(b->layer & a->mask)) {
                continue;
            }
            sip_push_pair(runtime, i, j);
        }
    }
}

void sip_broadphase(SipCollisionRuntime *runtime) {
    sip_radix_sort(runtime);
    sip_generate_pairs(runtime);
}
