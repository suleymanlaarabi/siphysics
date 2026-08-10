#include "internal.h"
#include "siecs.h"
#include "siphysics/components.h"
#include "siphysics/physics.h"
#include <stdint.h>

static void apply_gravity(ecs_iter_t *it) {
    Velocity *restrict velocities = ecs_field(it, 0);
    const SipSettings *settings = ecs_get_resource_read(SipSettings);
    const float delta_time = (float)it->delta_time;

    for (uint32_t i = 0; i < it->count; i++) {
        velocities[i].x += settings->gravity_x * delta_time;
        velocities[i].y += settings->gravity_y * delta_time;
    }
}

void integrate_velocity(ecs_iter_t *it) {
    Position *restrict positions = ecs_field(it, 0);
    const Velocity *restrict velocities = ecs_field(it, 1);

    const uint32_t count = it->count;
    const float delta_time = (float)it->delta_time;

    for (uint32_t i = 0; i < count; i++) {
        positions[i].x += velocities[i].x * delta_time;
        positions[i].y += velocities[i].y * delta_time;
    }
}

void integrate_angular_velocity(ecs_iter_t *it) {
    Rotation *restrict rotations = ecs_field(it, 0);
    const AngularVelocity *restrict angular_velocities = ecs_field(it, 1);

    const uint32_t count = it->count;
    const float delta_time = (float)it->delta_time;

    for (uint32_t i = 0; i < count; i++) {
        rotations[i].angle += angular_velocities[i].value * delta_time;
    }
}

void siphysics_register_systems(void) {
    const ecs_system_id_t gravity_system = ecs_system(
        {
            .name = "ApplyGravity",
            .query.terms = {
                ecs_inout(Velocity),
                ecs_filter(Dynamic),
                ecs_not(Kinematic),
                ecs_not(Static),
            },
            .phase = siphysics_phase,
            .callback = apply_gravity,
            .main_thread_only = true,
        }
    );

    const ecs_system_id_t velocity_system = ecs_system(
        {
            .name = "IntegrateVelocity",
            .query.terms = { ecs_inout(Position), ecs_in(Velocity) },
            .phase = siphysics_phase,
            .after = { gravity_system },
            .callback = integrate_velocity,
            .main_thread_only = true,
        }
    );

    const ecs_system_id_t angular_velocity_system = ecs_system(
        {
            .name = "IntegrateAngularVelocity",
            .query.terms = { ecs_inout(Rotation), ecs_in(AngularVelocity) },
            .phase = siphysics_phase,
            .callback = integrate_angular_velocity,
            .main_thread_only = true,
        }
    );

    siphysics_collision_register_system(velocity_system, angular_velocity_system);
}
