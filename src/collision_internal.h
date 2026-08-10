#ifndef SIPHYSICS_COLLISION_INTERNAL_H
#define SIPHYSICS_COLLISION_INTERNAL_H

#include <stdbool.h>

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
