#include "collision_internal.h"

typedef struct SipClipVertex {
    float x;
    float y;
    uint8_t incident_vertex;
} SipClipVertex;

static uint32_t sip_clip_segment_to_line(
    SipClipVertex out[2],
    const SipClipVertex in[2],
    float normal_x,
    float normal_y,
    float offset
) {
    const float distance_a = in[0].x * normal_x + in[0].y * normal_y - offset;
    const float distance_b = in[1].x * normal_x + in[1].y * normal_y - offset;
    const bool inside_a = distance_a <= 0.0f;
    const bool inside_b = distance_b <= 0.0f;
    uint32_t count = 0;
    if (inside_a) {
        out[count++] = in[0];
    }
    if (inside_b) {
        out[count++] = in[1];
    }
    const bool crosses =
        (distance_a < 0.0f && distance_b > 0.0f) ||
        (distance_a > 0.0f && distance_b < 0.0f);
    if (crosses) {
        const float fraction = distance_a / (distance_a - distance_b);
        SipClipVertex intersection = {
            .x = in[0].x + (in[1].x - in[0].x) * fraction,
            .y = in[0].y + (in[1].y - in[0].y) * fraction,
            .incident_vertex = inside_a ? in[1].incident_vertex : in[0].incident_vertex,
        };
        if (distance_a < 0.0f) {
            out[1] = intersection;
        } else {
            out[0] = intersection;
            out[1] = in[1];
        }
        return 2;
    }
    return count;
}

static void sip_obb_face_vertices(
    const SipObb *box,
    uint32_t face,
    SipClipVertex out[2]
) {
    float face_center_x;
    float face_center_y;
    float tangent_x;
    float tangent_y;
    float half_length;
    if (face == 0 || face == 2) {
        const float sign = face == 0 ? 1.0f : -1.0f;
        face_center_x = box->center_x + box->axis_x_x * box->half_width * sign;
        face_center_y = box->center_y + box->axis_x_y * box->half_width * sign;
        tangent_x = box->axis_y_x;
        tangent_y = box->axis_y_y;
        half_length = box->half_height;
    } else {
        const float sign = face == 1 ? 1.0f : -1.0f;
        face_center_x = box->center_x + box->axis_y_x * box->half_height * sign;
        face_center_y = box->center_y + box->axis_y_y * box->half_height * sign;
        tangent_x = box->axis_x_x;
        tangent_y = box->axis_x_y;
        half_length = box->half_width;
    }
    out[0] = (SipClipVertex){
        .x = face_center_x - tangent_x * half_length,
        .y = face_center_y - tangent_y * half_length,
        .incident_vertex = 0,
    };
    out[1] = (SipClipVertex){
        .x = face_center_x + tangent_x * half_length,
        .y = face_center_y + tangent_y * half_length,
        .incident_vertex = 1,
    };
}

static void sip_obb_face_normal(
    const SipObb *box,
    uint32_t face,
    float *out_x,
    float *out_y
) {
    if (face == 0 || face == 2) {
        const float sign = face == 0 ? 1.0f : -1.0f;
        *out_x = box->axis_x_x * sign;
        *out_y = box->axis_x_y * sign;
    } else {
        const float sign = face == 1 ? 1.0f : -1.0f;
        *out_x = box->axis_y_x * sign;
        *out_y = box->axis_y_y * sign;
    }
}

static uint32_t sip_obb_face_for_axis(
    uint32_t axis,
    const SipObb *box,
    float normal_x,
    float normal_y
) {
    const float axis_x = axis == 0 ? box->axis_x_x : box->axis_y_x;
    const float axis_y = axis == 0 ? box->axis_x_y : box->axis_y_y;
    const uint32_t positive_face = axis == 0 ? 0u : 1u;
    const uint32_t negative_face = axis == 0 ? 2u : 3u;
    return sip_dot2(axis_x, axis_y, normal_x, normal_y) >= 0.0f
        ? positive_face
        : negative_face;
}

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
        out->point_count = 1;
        out->points[0].penetration = radius;
        out->points[0].x = ax + ar;
        out->points[0].y = ay;
        out->points[0].feature_id = 0;
        return true;
    }

    const float distance = sip_sqrtf(distance_squared);
    const float normal_x = dx / distance;
    const float normal_y = dy / distance;
    out->normal_x = normal_x;
    out->normal_y = normal_y;
    out->point_count = 1;
    out->points[0].penetration = radius - distance;
    out->points[0].x = (ax + normal_x * ar + bx - normal_x * br) * 0.5f;
    out->points[0].y = (ay + normal_y * ar + by - normal_y * br) * 0.5f;
    out->points[0].feature_id = 0;
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
        out->point_count = 1;
        out->points[0].penetration = radius - distance;
        out->points[0].x = (cx + normal_x * radius + closest_world_x) * 0.5f;
        out->points[0].y = (cy + normal_y * radius + closest_world_y) * 0.5f;
        if (sip_absf(local_x) >= box->half_width && sip_absf(local_y) >= box->half_height) {
            out->points[0].feature_id =
                local_x >= 0.0f
                    ? (local_y >= 0.0f ? 6u : 5u)
                    : (local_y >= 0.0f ? 7u : 4u);
        } else if (sip_absf(local_x) >= box->half_width) {
            out->points[0].feature_id = local_x >= 0.0f ? 0u : 2u;
        } else {
            out->points[0].feature_id = local_y >= 0.0f ? 1u : 3u;
        }
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
    out->point_count = 1;
    out->points[0].penetration = radius + face_distance;
    out->points[0].x = (cx + normal_x * radius + cx + normal_x * face_distance) * 0.5f;
    out->points[0].y = (cy + normal_y * radius + cy + normal_y * face_distance) * 0.5f;
    out->points[0].feature_id = distance_x <= distance_y
        ? (local_x >= 0.0f ? 0u : 2u)
        : (local_y >= 0.0f ? 1u : 3u);
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
    const float axis_bias = 0.001f;
    float penetration = sip_box_overlap_on_axis(a, b, a->axis_x_x, a->axis_x_y);
    if (penetration < 0.0f) {
        return false;
    }
    float normal_x = a->axis_x_x;
    float normal_y = a->axis_x_y;
    uint32_t reference_axis = 0;
    bool reference_is_b = false;

    float overlap = sip_box_overlap_on_axis(a, b, a->axis_y_x, a->axis_y_y);
    if (overlap < 0.0f) {
        return false;
    }
    if (overlap < penetration - axis_bias) {
        penetration = overlap;
        normal_x = a->axis_y_x;
        normal_y = a->axis_y_y;
        reference_axis = 1;
    }

    overlap = sip_box_overlap_on_axis(a, b, b->axis_x_x, b->axis_x_y);
    if (overlap < 0.0f) {
        return false;
    }
    if (overlap < penetration - axis_bias) {
        penetration = overlap;
        normal_x = b->axis_x_x;
        normal_y = b->axis_x_y;
        reference_axis = 0;
        reference_is_b = true;
    }

    overlap = sip_box_overlap_on_axis(a, b, b->axis_y_x, b->axis_y_y);
    if (overlap < 0.0f) {
        return false;
    }
    if (overlap < penetration - axis_bias) {
        penetration = overlap;
        normal_x = b->axis_y_x;
        normal_y = b->axis_y_y;
        reference_axis = 1;
        reference_is_b = true;
    }

    const float center_delta_x = b->center_x - a->center_x;
    const float center_delta_y = b->center_y - a->center_y;
    if (sip_dot2(center_delta_x, center_delta_y, normal_x, normal_y) < 0.0f) {
        normal_x = -normal_x;
        normal_y = -normal_y;
    }

    const SipObb *reference_box = reference_is_b ? b : a;
    const SipObb *incident_box = reference_is_b ? a : b;
    const float reference_normal_x = reference_is_b ? -normal_x : normal_x;
    const float reference_normal_y = reference_is_b ? -normal_y : normal_y;
    const uint32_t reference_face = sip_obb_face_for_axis(
        reference_axis, reference_box, reference_normal_x, reference_normal_y);
    float best_dot = 1.0f;
    uint32_t incident_face = 0;
    for (uint32_t face = 0; face < 4; face++) {
        float face_normal_x;
        float face_normal_y;
        sip_obb_face_normal(incident_box, face, &face_normal_x, &face_normal_y);
        const float dot = sip_dot2(face_normal_x, face_normal_y,
                                   reference_normal_x, reference_normal_y);
        if (dot < best_dot) {
            best_dot = dot;
            incident_face = face;
        }
    }

    SipClipVertex reference_vertices[2];
    SipClipVertex incident_vertices[2];
    sip_obb_face_vertices(reference_box, reference_face, reference_vertices);
    sip_obb_face_vertices(incident_box, incident_face, incident_vertices);
    const float tangent_x = reference_vertices[1].x - reference_vertices[0].x;
    const float tangent_y = reference_vertices[1].y - reference_vertices[0].y;
    const float tangent_length = sip_sqrtf(tangent_x * tangent_x + tangent_y * tangent_y);
    const float side_normal_x = tangent_x / tangent_length;
    const float side_normal_y = tangent_y / tangent_length;
    const float left_normal_x = -side_normal_x;
    const float left_normal_y = -side_normal_y;
    const float left_offset = left_normal_x * reference_vertices[0].x +
                              left_normal_y * reference_vertices[0].y;
    const float right_offset = side_normal_x * reference_vertices[1].x +
                               side_normal_y * reference_vertices[1].y;
    SipClipVertex clip_a[2];
    SipClipVertex clip_b[2];
    uint32_t clip_count = sip_clip_segment_to_line(
        clip_a, incident_vertices, left_normal_x, left_normal_y, left_offset);
    if (clip_count == 0) {
        return false;
    }
    clip_count = sip_clip_segment_to_line(
        clip_b, clip_a, side_normal_x, side_normal_y, right_offset);
    if (clip_count == 0) {
        return false;
    }

    const float front_offset = reference_normal_x * reference_vertices[0].x +
                               reference_normal_y * reference_vertices[0].y;
    out->normal_x = normal_x;
    out->normal_y = normal_y;
    out->point_count = 0;
    for (uint32_t i = 0; i < clip_count && out->point_count < SIP_MAX_MANIFOLD_POINTS; i++) {
        const float separation = reference_normal_x * clip_b[i].x +
                                 reference_normal_y * clip_b[i].y - front_offset;
        if (separation <= 0.0f) {
            SipContactPoint *point = &out->points[out->point_count++];
            point->x = clip_b[i].x - reference_normal_x * separation * 0.5f;
            point->y = clip_b[i].y - reference_normal_y * separation * 0.5f;
            point->penetration = -separation > 0.0f ? -separation : 0.0f;
            point->feature_id = sip_contact_feature_id(
                reference_face, incident_face, clip_b[i].incident_vertex, reference_is_b);
        }
    }
    return out->point_count != 0;
}
