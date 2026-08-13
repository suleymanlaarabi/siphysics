#include "internal.h"
#include "siecs.h"
#include "siphysics/components.h"
#include "siphysics/physics.h"
#include <stdint.h>

static void sip_apply_gravity(
    ecs_query_id_t query,
    const SipSettings *settings,
    float dt
) {
    ecs_iter_t it = ecs_query_iter(query);

    while (ecs_iter_next(&it)) {
        Velocity *restrict velocities = ecs_field(&it, 0);

        for (uint32_t i = 0; i < it.count; i++) {
            velocities[i].x += settings->gravity_x * dt;
            velocities[i].y += settings->gravity_y * dt;
        }
    }
}

static void sip_integrate_velocity(
    ecs_query_id_t query,
    float dt
) {
    ecs_iter_t it = ecs_query_iter(query);

    while (ecs_iter_next(&it)) {
        Position *restrict positions = ecs_field(&it, 0);
        const Velocity *restrict velocities = ecs_field(&it, 1);

        for (uint32_t i = 0; i < it.count; i++) {
            positions[i].x += velocities[i].x * dt;
            positions[i].y += velocities[i].y * dt;
        }
    }
}

static void sip_integrate_angular_velocity(
    ecs_query_id_t query,
    float dt
) {
    ecs_iter_t it = ecs_query_iter(query);

    while (ecs_iter_next(&it)) {
        Rotation *restrict rotations = ecs_field(&it, 0);
        const AngularVelocity *restrict angular_velocities =
            ecs_field(&it, 1);

        for (uint32_t i = 0; i < it.count; i++) {
            rotations[i].angle +=
                angular_velocities[i].value * dt;
        }
    }
}

static void sip_fixed_step(
    SipPhysicsRuntime *runtime,
    const SipSettings *settings,
    float dt
) {
    sip_apply_gravity(
        runtime->gravity_query,
        settings,
        dt
    );

    sip_integrate_velocity(
        runtime->velocity_query,
        dt
    );

    sip_integrate_angular_velocity(
        runtime->angular_velocity_query,
        dt
    );

    siphysics_collision_step();
}

void siphysics_advance(float frame_dt) {
    if (!(frame_dt > 0.0f)) {
        return;
    }

    SipPhysicsRuntime *runtime =
        ecs_get_resource(SipPhysicsRuntime);

    const SipSettings *settings =
        ecs_get_resource_read(SipSettings);

    double frame = (double)frame_dt;
    const double max_frame_dt =
        (double)settings->max_frame_dt;
    const double fixed_dt =
        (double)settings->fixed_dt;

    if (frame > max_frame_dt) {
        frame = max_frame_dt;
    }

    runtime->accumulator += frame;

    uint32_t steps = 0;

    while (
        runtime->accumulator >= fixed_dt &&
        steps < settings->max_substeps
    ) {
        sip_fixed_step(
            runtime,
            settings,
            settings->fixed_dt
        );

        runtime->accumulator -= fixed_dt;
        runtime->tick++;
        steps++;
    }

    if (
        steps == settings->max_substeps &&
        runtime->accumulator >= fixed_dt
    ) {
        const uint64_t pending_steps =
            (uint64_t)(runtime->accumulator / fixed_dt);

        runtime->accumulator -=
            (double)pending_steps * fixed_dt;
    }
}

static void sip_physics_step_system(ecs_iter_t *it) {
    siphysics_advance(it->delta_time);
}

void siphysics_register_systems(void) {
    SipPhysicsRuntime *runtime =
        ecs_get_resource(SipPhysicsRuntime);

    runtime->gravity_query =
        ecs_query_init(&(ecs_query_desc_t){
            .terms = {
                ecs_inout(Velocity),
                ecs_filter(Dynamic),
                ecs_not(Kinematic),
                ecs_not(Static),
            },
        });

    runtime->velocity_query =
        ecs_query_init(&(ecs_query_desc_t){
            .terms = {
                ecs_inout(Position),
                ecs_in(Velocity),
            },
        });

    runtime->angular_velocity_query =
        ecs_query_init(&(ecs_query_desc_t){
            .terms = {
                ecs_inout(Rotation),
                ecs_in(AngularVelocity),
            },
        });

    ecs_system({
        .name = "PhysicsStep",
        .phase = siphysics_phase,
        .callback = sip_physics_step_system,
        .read_resources = {
            ecs_id(SipSettings),
            ecs_id(SipCollisionCapacity),
        },
        .write_resources = {
            ecs_id(SipPhysicsRuntime),
            ecs_id(SipCollisionRuntime),
            ecs_id(SipCollisionStats),
        },
        .main_thread_only = true,
    });
}
