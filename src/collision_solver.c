#include "collision_runtime.h"

static float sip_body_inverse_mass(const SipBatchRef *batch, uint32_t row) {
    return batch->body_type == SIP_BODY_DYNAMIC ? batch->inverse_masses[row].value : 0.0f;
}

static void sip_apply_position(Position *position, uint8_t body_type, float x, float y) {
    if (body_type == SIP_BODY_DYNAMIC) {
        position->x += x;
        position->y += y;
    }
}

static void sip_apply_velocity(Velocity *velocity, uint8_t body_type, float x, float y) {
    if (body_type == SIP_BODY_DYNAMIC && velocity) {
        velocity->x += x;
        velocity->y += y;
    }
}

static inline void sip_relative_velocity(const SipBatchRef *batch_a, uint32_t row_a,
                                         const SipBatchRef *batch_b, uint32_t row_b,
                                         float *restrict out_x, float *restrict out_y) {
    const float velocity_a_x = batch_a->velocities ? batch_a->velocities[row_a].x : 0.0f;
    const float velocity_a_y = batch_a->velocities ? batch_a->velocities[row_a].y : 0.0f;
    const float velocity_b_x = batch_b->velocities ? batch_b->velocities[row_b].x : 0.0f;
    const float velocity_b_y = batch_b->velocities ? batch_b->velocities[row_b].y : 0.0f;
    *out_x = velocity_b_x - velocity_a_x;
    *out_y = velocity_b_y - velocity_a_y;
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

static void sip_collision_solve_velocities(SipCollisionRuntime *runtime, const SipSettings *settings) {
    for (uint32_t iteration = 0; iteration < settings->solver_iterations; iteration++) {
        for (uint32_t i = 0; i < runtime->contact_count; i++) {
            SipSolverContact *contact = &runtime->contacts[i];
            if (contact->sensor) continue;
            const SipBatchRef *batch_a = &runtime->batches[contact->batch_a];
            const SipBatchRef *batch_b = &runtime->batches[contact->batch_b];
            const float inverse_mass_a = sip_body_inverse_mass(batch_a, contact->row_a);
            const float inverse_mass_b = sip_body_inverse_mass(batch_b, contact->row_b);
            const float inverse_mass_sum = inverse_mass_a + inverse_mass_b;
            if (!(inverse_mass_sum > 0.0f)) continue;
            float relative_x, relative_y;
            sip_relative_velocity(batch_a, contact->row_a, batch_b, contact->row_b, &relative_x, &relative_y);
            const float relative_normal = relative_x * contact->normal_x + relative_y * contact->normal_y;
            const float restitution = relative_normal < -settings->restitution_threshold ? contact->restitution : 0.0f;
            const float normal_lambda = relative_normal > 0.0f
                ? 0.0f
                : -(1.0f + restitution) * relative_normal / inverse_mass_sum;
            const float old_normal_impulse = contact->normal_impulse;
            float new_normal_impulse = old_normal_impulse + normal_lambda;
            if (new_normal_impulse < 0.0f) new_normal_impulse = 0.0f;
            contact->normal_impulse = new_normal_impulse;
            const float normal_impulse_delta = new_normal_impulse - old_normal_impulse;
            sip_apply_velocity(batch_a->velocities ? &batch_a->velocities[contact->row_a] : NULL, contact->body_type_a,
                               -contact->normal_x * normal_impulse_delta * inverse_mass_a,
                               -contact->normal_y * normal_impulse_delta * inverse_mass_a);
            sip_apply_velocity(batch_b->velocities ? &batch_b->velocities[contact->row_b] : NULL, contact->body_type_b,
                               contact->normal_x * normal_impulse_delta * inverse_mass_b,
                               contact->normal_y * normal_impulse_delta * inverse_mass_b);
            sip_relative_velocity(batch_a, contact->row_a, batch_b, contact->row_b, &relative_x, &relative_y);
            const float tangent_x = -contact->normal_y, tangent_y = contact->normal_x;
            const float tangent_lambda = -(relative_x * tangent_x + relative_y * tangent_y) / inverse_mass_sum;
            const float tangent_limit = contact->friction * contact->normal_impulse;
            const float old_tangent_impulse = contact->tangent_impulse;
            const float new_tangent_impulse = sip_clampf(old_tangent_impulse + tangent_lambda,
                                                          -tangent_limit, tangent_limit);
            contact->tangent_impulse = new_tangent_impulse;
            const float tangent_impulse_delta = new_tangent_impulse - old_tangent_impulse;
            sip_apply_velocity(batch_a->velocities ? &batch_a->velocities[contact->row_a] : NULL, contact->body_type_a,
                               -tangent_x * tangent_impulse_delta * inverse_mass_a,
                               -tangent_y * tangent_impulse_delta * inverse_mass_a);
            sip_apply_velocity(batch_b->velocities ? &batch_b->velocities[contact->row_b] : NULL, contact->body_type_b,
                               tangent_x * tangent_impulse_delta * inverse_mass_b,
                               tangent_y * tangent_impulse_delta * inverse_mass_b);
        }
    }
}

void sip_collision_solve(SipCollisionRuntime *runtime, const SipSettings *settings) {
    sip_collision_correct_positions(runtime, settings);
    sip_collision_solve_velocities(runtime, settings);
}
