#include "collision_runtime.h"

static inline float sip_body_inverse_mass(const SipBatchRef *batch, uint32_t row) {
    return batch->body_type == SIP_BODY_DYNAMIC ? batch->inverse_masses[row].value : 0.0f;
}

static inline float sip_body_inverse_inertia(const SipBatchRef *batch, uint32_t row) {
    return batch->body_type == SIP_BODY_DYNAMIC && batch->inverse_inertias
        ? batch->inverse_inertias[row].value
        : 0.0f;
}

static inline float sip_cross2(float ax, float ay, float bx, float by) {
    return ax * by - ay * bx;
}

static void sip_apply_position(Position *position, uint8_t body_type, float x, float y) {
    if (body_type == SIP_BODY_DYNAMIC) {
        position->x += x;
        position->y += y;
    }
}

static inline void sip_contact_velocity(
    const SipBatchRef *batch,
    uint32_t row,
    float point_x,
    float point_y,
    float *restrict out_x,
    float *restrict out_y
) {
    float vx = batch->velocities ? batch->velocities[row].x : 0.0f;
    float vy = batch->velocities ? batch->velocities[row].y : 0.0f;

    if (batch->angular_velocities) {
        const float rx = point_x - batch->positions[row].x;
        const float ry = point_y - batch->positions[row].y;
        const float omega = batch->angular_velocities[row].value;
        vx -= omega * ry;
        vy += omega * rx;
    }

    *out_x = vx;
    *out_y = vy;
}

static inline void sip_relative_contact_velocity(
    const SipBatchRef *batch_a,
    uint32_t row_a,
    const SipBatchRef *batch_b,
    uint32_t row_b,
    float point_x,
    float point_y,
    float *restrict out_x,
    float *restrict out_y
) {
    float velocity_a_x;
    float velocity_a_y;
    float velocity_b_x;
    float velocity_b_y;
    sip_contact_velocity(batch_a, row_a, point_x, point_y, &velocity_a_x, &velocity_a_y);
    sip_contact_velocity(batch_b, row_b, point_x, point_y, &velocity_b_x, &velocity_b_y);
    *out_x = velocity_b_x - velocity_a_x;
    *out_y = velocity_b_y - velocity_a_y;
}

static inline void sip_apply_impulse(
    SipBatchRef *batch,
    uint32_t row,
    uint8_t body_type,
    float point_x,
    float point_y,
    float impulse_x,
    float impulse_y
) {
    if (body_type != SIP_BODY_DYNAMIC) {
        return;
    }

    const float inverse_mass = batch->inverse_masses[row].value;
    batch->velocities[row].x += impulse_x * inverse_mass;
    batch->velocities[row].y += impulse_y * inverse_mass;

    const float rx = point_x - batch->positions[row].x;
    const float ry = point_y - batch->positions[row].y;
    batch->angular_velocities[row].value +=
        batch->inverse_inertias[row].value * sip_cross2(rx, ry, impulse_x, impulse_y);
}

static void sip_collision_correct_positions(SipCollisionRuntime *runtime, const SipSettings *settings) {
    for (uint32_t i = 0; i < runtime->contact_count; i++) {
        SipSolverContact *contact = &runtime->contacts[i];
        if (contact->sensor) continue;
        const SipBatchRef *batch_a = &runtime->batches[contact->batch_a];
        const SipBatchRef *batch_b = &runtime->batches[contact->batch_b];
        const float inverse_mass_a = sip_body_inverse_mass(batch_a, contact->row_a);
        const float inverse_mass_b = sip_body_inverse_mass(batch_b, contact->row_b);
        const float inverse_mass_sum = inverse_mass_a + inverse_mass_b;
        if (!(inverse_mass_sum > 0.0f)) continue;
        const float correction_depth = contact->penetration - settings->penetration_slop;
        if (!(correction_depth > 0.0f)) continue;
        const float correction = correction_depth * settings->penetration_correction / inverse_mass_sum;
        sip_apply_position(&batch_a->positions[contact->row_a], contact->body_type_a,
                           -contact->normal_x * correction * inverse_mass_a,
                           -contact->normal_y * correction * inverse_mass_a);
        sip_apply_position(&batch_b->positions[contact->row_b], contact->body_type_b,
                           contact->normal_x * correction * inverse_mass_b,
                           contact->normal_y * correction * inverse_mass_b);
    }
}

static void sip_collision_prepare_velocities(
    SipCollisionRuntime *runtime,
    const SipSettings *settings
) {
    for (uint32_t i = 0; i < runtime->contact_count; i++) {
        SipSolverContact *contact = &runtime->contacts[i];
        contact->restitution_bias = 0.0f;
        contact->normal_mass = 0.0f;
        contact->tangent_mass = 0.0f;

        if (contact->sensor) continue;

        const SipBatchRef *batch_a = &runtime->batches[contact->batch_a];
        const SipBatchRef *batch_b = &runtime->batches[contact->batch_b];
        const float inverse_mass_a = sip_body_inverse_mass(batch_a, contact->row_a);
        const float inverse_mass_b = sip_body_inverse_mass(batch_b, contact->row_b);
        const float inverse_inertia_a = sip_body_inverse_inertia(batch_a, contact->row_a);
        const float inverse_inertia_b = sip_body_inverse_inertia(batch_b, contact->row_b);

        const float ra_x = contact->point_x - batch_a->positions[contact->row_a].x;
        const float ra_y = contact->point_y - batch_a->positions[contact->row_a].y;
        const float rb_x = contact->point_x - batch_b->positions[contact->row_b].x;
        const float rb_y = contact->point_y - batch_b->positions[contact->row_b].y;
        const float rn_a = sip_cross2(ra_x, ra_y, contact->normal_x, contact->normal_y);
        const float rn_b = sip_cross2(rb_x, rb_y, contact->normal_x, contact->normal_y);
        const float k_normal = inverse_mass_a + inverse_mass_b +
                               inverse_inertia_a * rn_a * rn_a +
                               inverse_inertia_b * rn_b * rn_b;
        contact->normal_mass = k_normal > 0.000001f ? 1.0f / k_normal : 0.0f;

        const float tangent_x = -contact->normal_y;
        const float tangent_y = contact->normal_x;
        const float rt_a = sip_cross2(ra_x, ra_y, tangent_x, tangent_y);
        const float rt_b = sip_cross2(rb_x, rb_y, tangent_x, tangent_y);
        const float k_tangent = inverse_mass_a + inverse_mass_b +
                                inverse_inertia_a * rt_a * rt_a +
                                inverse_inertia_b * rt_b * rt_b;
        contact->tangent_mass = k_tangent > 0.000001f ? 1.0f / k_tangent : 0.0f;

        if (!(contact->normal_mass > 0.0f)) continue;

        float relative_x;
        float relative_y;
        sip_relative_contact_velocity(
            batch_a, contact->row_a, batch_b, contact->row_b,
            contact->point_x, contact->point_y, &relative_x, &relative_y);
        const float relative_normal = relative_x * contact->normal_x +
                                      relative_y * contact->normal_y;
        if (relative_normal < -settings->restitution_threshold) {
            contact->restitution_bias = -contact->restitution * relative_normal;
        }
    }
}

static void sip_collision_warm_start(SipCollisionRuntime *runtime) {
    for (uint32_t i = 0; i < runtime->contact_count; i++) {
        SipSolverContact *contact = &runtime->contacts[i];
        if (contact->sensor) continue;
        if (contact->normal_impulse == 0.0f && contact->tangent_impulse == 0.0f) continue;

        SipBatchRef *batch_a = &runtime->batches[contact->batch_a];
        SipBatchRef *batch_b = &runtime->batches[contact->batch_b];
        const float tangent_x = -contact->normal_y;
        const float tangent_y = contact->normal_x;
        const float impulse_x = contact->normal_x * contact->normal_impulse +
                                tangent_x * contact->tangent_impulse;
        const float impulse_y = contact->normal_y * contact->normal_impulse +
                                tangent_y * contact->tangent_impulse;

        sip_apply_impulse(batch_a, contact->row_a, contact->body_type_a,
                          contact->point_x, contact->point_y, -impulse_x, -impulse_y);
        sip_apply_impulse(batch_b, contact->row_b, contact->body_type_b,
                          contact->point_x, contact->point_y, impulse_x, impulse_y);
    }
}

static void sip_collision_solve_velocities(SipCollisionRuntime *runtime, const SipSettings *settings) {
    for (uint32_t iteration = 0; iteration < settings->solver_iterations; iteration++) {
        for (uint32_t i = 0; i < runtime->contact_count; i++) {
            SipSolverContact *contact = &runtime->contacts[i];
            if (contact->sensor || !(contact->normal_mass > 0.0f)) continue;
            SipBatchRef *batch_a = &runtime->batches[contact->batch_a];
            SipBatchRef *batch_b = &runtime->batches[contact->batch_b];

            float relative_x;
            float relative_y;
            sip_relative_contact_velocity(
                batch_a, contact->row_a, batch_b, contact->row_b,
                contact->point_x, contact->point_y, &relative_x, &relative_y);
            const float relative_normal = relative_x * contact->normal_x +
                                          relative_y * contact->normal_y;
            const float normal_lambda =
                -(relative_normal - contact->restitution_bias) * contact->normal_mass;
            const float old_normal_impulse = contact->normal_impulse;
            float new_normal_impulse = old_normal_impulse + normal_lambda;
            if (new_normal_impulse < 0.0f) new_normal_impulse = 0.0f;
            contact->normal_impulse = new_normal_impulse;
            const float normal_impulse_delta = new_normal_impulse - old_normal_impulse;
            sip_apply_impulse(batch_a, contact->row_a, contact->body_type_a,
                              contact->point_x, contact->point_y,
                              -contact->normal_x * normal_impulse_delta,
                              -contact->normal_y * normal_impulse_delta);
            sip_apply_impulse(batch_b, contact->row_b, contact->body_type_b,
                              contact->point_x, contact->point_y,
                              contact->normal_x * normal_impulse_delta,
                              contact->normal_y * normal_impulse_delta);

            sip_relative_contact_velocity(
                batch_a, contact->row_a, batch_b, contact->row_b,
                contact->point_x, contact->point_y, &relative_x, &relative_y);
            const float tangent_x = -contact->normal_y;
            const float tangent_y = contact->normal_x;
            const float tangent_lambda =
                -(relative_x * tangent_x + relative_y * tangent_y) * contact->tangent_mass;
            const float tangent_limit = contact->friction * contact->normal_impulse;
            const float old_tangent_impulse = contact->tangent_impulse;
            const float new_tangent_impulse = sip_clampf(
                old_tangent_impulse + tangent_lambda, -tangent_limit, tangent_limit);
            contact->tangent_impulse = new_tangent_impulse;
            const float tangent_impulse_delta = new_tangent_impulse - old_tangent_impulse;
            sip_apply_impulse(batch_a, contact->row_a, contact->body_type_a,
                              contact->point_x, contact->point_y,
                              -tangent_x * tangent_impulse_delta,
                              -tangent_y * tangent_impulse_delta);
            sip_apply_impulse(batch_b, contact->row_b, contact->body_type_b,
                              contact->point_x, contact->point_y,
                              tangent_x * tangent_impulse_delta,
                              tangent_y * tangent_impulse_delta);
        }
    }
}

void sip_collision_solve(SipCollisionRuntime *runtime, const SipSettings *settings) {
    sip_collision_correct_positions(runtime, settings);
    sip_collision_prepare_velocities(runtime, settings);
    sip_collision_warm_start(runtime);
    sip_collision_solve_velocities(runtime, settings);
    sip_contact_cache_store(runtime);
}
