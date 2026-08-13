#ifndef SIPHYSICS_COLLISION_INTERNAL_H
#define SIPHYSICS_COLLISION_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#if defined(__SSE__)
#include <xmmintrin.h>
#endif

#define SIP_MAX_MANIFOLD_POINTS 2

typedef struct SipContactPoint {
    float x;
    float y;
    float penetration;
    uint32_t feature_id;
} SipContactPoint;

typedef struct SipContact {
    float normal_x;
    float normal_y;

    SipContactPoint points[SIP_MAX_MANIFOLD_POINTS];
    uint8_t point_count;
} SipContact;

static inline uint32_t sip_contact_feature_id(
    uint32_t reference_face,
    uint32_t incident_face,
    uint32_t incident_vertex,
    bool flip
) {
    return
        (reference_face & 3u) |
        ((incident_face & 3u) << 2) |
        ((incident_vertex & 1u) << 4) |
        ((flip ? 1u : 0u) << 5);
}

typedef struct SipObb {
    float center_x;
    float center_y;

    float half_width;
    float half_height;

    float axis_x_x;
    float axis_x_y;

    float axis_y_x;
    float axis_y_y;
} SipObb;

static inline float sip_dot2(float ax, float ay, float bx, float by) {
    return ax * bx + ay * by;
}

static inline float sip_clampf(float value, float minimum, float maximum) {
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

static inline float sip_absf(float value) {
    return value < 0.0f ? -value : value;
}

static inline float sip_sqrtf(float value) {
#if defined(__SSE__)
    return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ss(value)));
#else
    float estimate = value > 1.0f ? value : 1.0f;
    for (uint32_t i = 0; i < 8; i++) {
        estimate = (estimate + value / estimate) * 0.5f;
    }
    return estimate;
#endif
}

static inline void sip_sincosf(float angle, float *restrict sine, float *restrict cosine) {
    const float half_pi = 1.57079632679489661923f;
    const int32_t quadrant = (int32_t)(angle / half_pi + (angle >= 0.0f ? 0.5f : -0.5f));
    const float x = angle - (float)quadrant * half_pi;
    const float x2 = x * x;
    const float x4 = x2 * x2;
    const float x6 = x4 * x2;
    const float x8 = x4 * x4;
    const float base_sine = x * (1.0f - x2 * (1.0f / 6.0f) + x4 * (1.0f / 120.0f) - x6 * (1.0f / 5040.0f));
    const float base_cosine = 1.0f - x2 * 0.5f + x4 * (1.0f / 24.0f) - x6 * (1.0f / 720.0f) + x8 * (1.0f / 40320.0f);
    switch (quadrant & 3) {
        case 0: *sine = base_sine; *cosine = base_cosine; break;
        case 1: *sine = base_cosine; *cosine = -base_sine; break;
        case 2: *sine = -base_sine; *cosine = -base_cosine; break;
        default: *sine = -base_cosine; *cosine = base_sine; break;
    }
}

bool sip_circle_circle(
    float ax,
    float ay,
    float ar,
    float bx,
    float by,
    float br,
    SipContact *restrict out
);

bool sip_circle_box(
    float cx,
    float cy,
    float radius,
    const SipObb *restrict box,
    SipContact *restrict out
);

bool sip_box_box(
    const SipObb *restrict a,
    const SipObb *restrict b,
    SipContact *restrict out
);

#endif
