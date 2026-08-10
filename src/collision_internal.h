#ifndef SIPHYSICS_COLLISION_INTERNAL_H
#define SIPHYSICS_COLLISION_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#if defined(__SSE__)
#include <xmmintrin.h>
#endif

typedef struct SipContact {
    float normal_x;
    float normal_y;
    float point_x;
    float point_y;
    float penetration;
} SipContact;

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

static inline void sip_obb_support(
    const SipObb *restrict box,
    float direction_x,
    float direction_y,
    float *restrict out_x,
    float *restrict out_y
) {
    const float x = sip_dot2(direction_x, direction_y, box->axis_x_x, box->axis_x_y) >= 0.0f
                        ? box->half_width
                        : -box->half_width;
    const float y = sip_dot2(direction_x, direction_y, box->axis_y_x, box->axis_y_y) >= 0.0f
                        ? box->half_height
                        : -box->half_height;
    *out_x = box->center_x + box->axis_x_x * x + box->axis_y_x * y;
    *out_y = box->center_y + box->axis_x_y * x + box->axis_y_y * y;
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
