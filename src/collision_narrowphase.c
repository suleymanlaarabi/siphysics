#include "collision_internal.h"

bool sip_circle_circle(
    float ax,
    float ay,
    float ar,
    float bx,
    float by,
    float br,
    SipContact *restrict out
) {
    const float dx = bx - ax;
    const float dy = by - ay;
    const float radius = ar + br;
    const float distance_squared = dx * dx + dy * dy;
    const float radius_squared = radius * radius;

    if (distance_squared > radius_squared) {
        return false;
    }

    if (distance_squared == 0.0f) {
        out->normal_x = 1.0f;
        out->normal_y = 0.0f;
        out->penetration = radius;
        out->point_x = ax + ar;
        out->point_y = ay;
        return true;
    }

    const float distance = sip_sqrtf(distance_squared);
    const float normal_x = dx / distance;
    const float normal_y = dy / distance;
    out->normal_x = normal_x;
    out->normal_y = normal_y;
    out->penetration = radius - distance;
    out->point_x = (ax + normal_x * ar + bx - normal_x * br) * 0.5f;
    out->point_y = (ay + normal_y * ar + by - normal_y * br) * 0.5f;
    return true;
}

bool sip_circle_box(
    float cx,
    float cy,
    float radius,
    const SipObb *restrict box,
    SipContact *restrict out
) {
    const float dx = cx - box->center_x;
    const float dy = cy - box->center_y;
    const float local_x = sip_dot2(dx, dy, box->axis_x_x, box->axis_x_y);
    const float local_y = sip_dot2(dx, dy, box->axis_y_x, box->axis_y_y);
    const float closest_x = sip_clampf(local_x, -box->half_width, box->half_width);
    const float closest_y = sip_clampf(local_y, -box->half_height, box->half_height);

    const float closest_world_x = box->center_x + box->axis_x_x * closest_x + box->axis_y_x * closest_y;
    const float closest_world_y = box->center_y + box->axis_x_y * closest_x + box->axis_y_y * closest_y;
    const float to_box_x = closest_world_x - cx;
    const float to_box_y = closest_world_y - cy;
    const float distance_squared = to_box_x * to_box_x + to_box_y * to_box_y;
    const float radius_squared = radius * radius;

    if (distance_squared > radius_squared) {
        return false;
    }

    if (distance_squared != 0.0f) {
        const float distance = sip_sqrtf(distance_squared);
        const float normal_x = to_box_x / distance;
        const float normal_y = to_box_y / distance;
        out->normal_x = normal_x;
        out->normal_y = normal_y;
        out->penetration = radius - distance;
        out->point_x = (cx + normal_x * radius + closest_world_x) * 0.5f;
        out->point_y = (cy + normal_y * radius + closest_world_y) * 0.5f;
        return true;
    }

    const float distance_x = box->half_width - sip_absf(local_x);
    const float distance_y = box->half_height - sip_absf(local_y);
    float normal_x;
    float normal_y;
    float face_distance;
    if (distance_x <= distance_y) {
        const float sign = local_x >= 0.0f ? 1.0f : -1.0f;
        normal_x = box->axis_x_x * sign;
        normal_y = box->axis_x_y * sign;
        face_distance = distance_x;
    } else {
        const float sign = local_y >= 0.0f ? 1.0f : -1.0f;
        normal_x = box->axis_y_x * sign;
        normal_y = box->axis_y_y * sign;
        face_distance = distance_y;
    }

    out->normal_x = normal_x;
    out->normal_y = normal_y;
    out->penetration = radius + face_distance;
    out->point_x = (cx + normal_x * radius + cx + normal_x * face_distance) * 0.5f;
    out->point_y = (cy + normal_y * radius + cy + normal_y * face_distance) * 0.5f;
    return true;
}

static inline float sip_box_overlap_on_axis(
    const SipObb *restrict a,
    const SipObb *restrict b,
    float axis_x,
    float axis_y
) {
    const float a_radius = a->half_width * sip_absf(sip_dot2(a->axis_x_x, a->axis_x_y, axis_x, axis_y))
                         + a->half_height * sip_absf(sip_dot2(a->axis_y_x, a->axis_y_y, axis_x, axis_y));
    const float b_radius = b->half_width * sip_absf(sip_dot2(b->axis_x_x, b->axis_x_y, axis_x, axis_y))
                         + b->half_height * sip_absf(sip_dot2(b->axis_y_x, b->axis_y_y, axis_x, axis_y));
    const float center_distance = sip_absf(
        sip_dot2(b->center_x - a->center_x, b->center_y - a->center_y, axis_x, axis_y)
    );
    return a_radius + b_radius - center_distance;
}

bool sip_box_box(
    const SipObb *restrict a,
    const SipObb *restrict b,
    SipContact *restrict out
) {
    float penetration = sip_box_overlap_on_axis(a, b, a->axis_x_x, a->axis_x_y);
    if (penetration < 0.0f) {
        return false;
    }
    float normal_x = a->axis_x_x;
    float normal_y = a->axis_x_y;

    float overlap = sip_box_overlap_on_axis(a, b, a->axis_y_x, a->axis_y_y);
    if (overlap < 0.0f) {
        return false;
    }
    if (overlap < penetration) {
        penetration = overlap;
        normal_x = a->axis_y_x;
        normal_y = a->axis_y_y;
    }

    overlap = sip_box_overlap_on_axis(a, b, b->axis_x_x, b->axis_x_y);
    if (overlap < 0.0f) {
        return false;
    }
    if (overlap < penetration) {
        penetration = overlap;
        normal_x = b->axis_x_x;
        normal_y = b->axis_x_y;
    }

    overlap = sip_box_overlap_on_axis(a, b, b->axis_y_x, b->axis_y_y);
    if (overlap < 0.0f) {
        return false;
    }
    if (overlap < penetration) {
        penetration = overlap;
        normal_x = b->axis_y_x;
        normal_y = b->axis_y_y;
    }

    const float center_delta_x = b->center_x - a->center_x;
    const float center_delta_y = b->center_y - a->center_y;
    if (sip_dot2(center_delta_x, center_delta_y, normal_x, normal_y) < 0.0f) {
        normal_x = -normal_x;
        normal_y = -normal_y;
    }

    float a_point_x;
    float a_point_y;
    float b_point_x;
    float b_point_y;
    sip_obb_support(a, normal_x, normal_y, &a_point_x, &a_point_y);
    sip_obb_support(b, -normal_x, -normal_y, &b_point_x, &b_point_y);
    out->normal_x = normal_x;
    out->normal_y = normal_y;
    out->penetration = penetration;
    out->point_x = (a_point_x + b_point_x) * 0.5f;
    out->point_y = (a_point_y + b_point_y) * 0.5f;
    return true;
}
