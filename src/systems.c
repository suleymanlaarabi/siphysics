#include "internal.h"
#include "siecs.h"
#include "siphysics/components.h"
#include "siphysics/physics.h"
#include <stdint.h>

void integrate_velocity(ecs_iter_t *it) {
    Position *restrict positions = ecs_field(it, 0);
    const Velocity *restrict velocities = ecs_field(it, 1);

    const uint32_t count = it->count;
    const float delta_time = it->delta_time;

    for (uint32_t i = 0; i < count; i++) {
        positions[i].x += velocities[i].x * delta_time;
        positions[i].y += velocities[i].y * delta_time;
    }
}

void integrate_angular_velocity(ecs_iter_t *it) {
    Rotation *restrict rotations = ecs_field(it, 0);
    const AngularVelocity *restrict angular_velocities = ecs_field(it, 1);

    const uint32_t count = it->count;
    const float delta_time = it->delta_time;

    for (uint32_t i = 0; i < count; i++) {
        rotations[i].angle += angular_velocities[i].value * delta_time;
    }
}

void siphysics_register_systems(void) {
    ecs_system(
        {
            .name = "IntegrateVelocity",
            .query.terms = { ecs_inout(Position), ecs_in(Velocity) },
            .phase = siphysics_phase,
            .callback = integrate_velocity,
        }
    );

    ecs_system(
        {
            .name = "IntegrateAngularVelocity",
            .query.terms = { ecs_inout(Rotation), ecs_in(AngularVelocity) },
            .phase = siphysics_phase,
            .callback = integrate_angular_velocity,
        }
    );
}
