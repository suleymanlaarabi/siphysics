#include "collision_runtime.h"

static float sip_body_inverse_mass(const SipBatchRef *batch, uint32_t row) {
    return batch->body_type == SIP_BODY_DYNAMIC ? batch->inverse_masses[row].value : 0.0f;
}

static void sip_apply_position(Position *position, uint8_t body_type,
                               float x, float y) {
    if (body_type == SIP_BODY_DYNAMIC) {
        position->x += x;
        position->y += y;
    }
}

static void sip_apply_velocity(Velocity *velocity, uint8_t body_type,
                               float x, float y) {
    if (body_type == SIP_BODY_DYNAMIC) {
        velocity->x += x;
        velocity->y += y;
    }
}

void sip_collision_solve(SipCollisionRuntime *runtime, const SipSettings *settings) {
    for (uint32_t iteration = 0; iteration < settings->solver_iterations; iteration++) {
        for (uint32_t i = 0; i < runtime->contact_count; i++) {
            SipSolverContact *contact = &runtime->contacts[i];
            if (contact->sensor) {
                continue;
            }

            const SipBatchRef *batch_a = &runtime->batches[contact->batch_a];
            const SipBatchRef *batch_b = &runtime->batches[contact->batch_b];
            const float inverse_mass_a = sip_body_inverse_mass(batch_a, contact->row_a);
            const float inverse_mass_b = sip_body_inverse_mass(batch_b, contact->row_b);
            const float inverse_mass_sum = inverse_mass_a + inverse_mass_b;
            if (inverse_mass_sum == 0.0f) {
                continue;
            }

            const float correction_depth = contact->penetration - settings->penetration_slop;
            if (correction_depth > 0.0f) {
                const float correction = correction_depth * settings->penetration_correction /
                                         inverse_mass_sum;
                sip_apply_position(
                    &batch_a->positions[contact->row_a], contact->body_type_a,
                    -contact->normal_x * correction * inverse_mass_a,
                    -contact->normal_y * correction * inverse_mass_a);
                sip_apply_position(
                    &batch_b->positions[contact->row_b], contact->body_type_b,
                    contact->normal_x * correction * inverse_mass_b,
                    contact->normal_y * correction * inverse_mass_b);
            }

            const Velocity zero_velocity = {0};
            const Velocity velocity_a = batch_a->velocities ? batch_a->velocities[contact->row_a]
                                                              : zero_velocity;
            const Velocity velocity_b = batch_b->velocities ? batch_b->velocities[contact->row_b]
                                                              : zero_velocity;
            const float relative_x = velocity_b.x - velocity_a.x;
            const float relative_y = velocity_b.y - velocity_a.y;
            const float relative_normal = relative_x * contact->normal_x +
                                           relative_y * contact->normal_y;
            if (relative_normal > 0.0f) {
                continue;
            }

            const float normal_impulse = -(1.0f + contact->restitution) * relative_normal /
                                         inverse_mass_sum;
            contact->normal_impulse += normal_impulse;
            sip_apply_velocity(
                batch_a->velocities ? &batch_a->velocities[contact->row_a] : NULL,
                contact->body_type_a,
                -contact->normal_x * normal_impulse * inverse_mass_a,
                -contact->normal_y * normal_impulse * inverse_mass_a);
            sip_apply_velocity(
                batch_b->velocities ? &batch_b->velocities[contact->row_b] : NULL,
                contact->body_type_b,
                contact->normal_x * normal_impulse * inverse_mass_b,
                contact->normal_y * normal_impulse * inverse_mass_b);

            const float tangent_x = -contact->normal_y;
            const float tangent_y = contact->normal_x;
            const float tangent_velocity = relative_x * tangent_x + relative_y * tangent_y;
            float tangent_impulse = -tangent_velocity / inverse_mass_sum;
            const float tangent_limit = contact->friction * normal_impulse;
            tangent_impulse = sip_clampf(tangent_impulse, -tangent_limit, tangent_limit);
            contact->tangent_impulse += tangent_impulse;
            sip_apply_velocity(
                batch_a->velocities ? &batch_a->velocities[contact->row_a] : NULL,
                contact->body_type_a,
                -tangent_x * tangent_impulse * inverse_mass_a,
                -tangent_y * tangent_impulse * inverse_mass_a);
            sip_apply_velocity(
                batch_b->velocities ? &batch_b->velocities[contact->row_b] : NULL,
                contact->body_type_b,
                tangent_x * tangent_impulse * inverse_mass_b,
                tangent_y * tangent_impulse * inverse_mass_b);
        }
    }
}
