#include "internal.h"
#include "siphysics/collision.h"
#include "siphysics/physics.h"

ECS_RESOURCE_DEFINE(SipSettings);
ECS_RESOURCE_DEFINE(SipCollisionStats);
ECS_RESOURCE_DEFINE(SipCollisionCapacity);
ECS_RESOURCE_DEFINE(SipCollisionRuntime, .ops.dtor = sip_collision_runtime_destroy);
ECS_RESOURCE_DEFINE(SipPhysicsRuntime);
ECS_MODULE_DEFINE(siphysics);

ecs_phase_t siphysics_phase = ECS_PHASE_NONE;

void siphysics_import(const siphysics_props_t *props) {
    ECS_COMPONENT_REGISTER(
        Position,
        Velocity,
        AngularVelocity,
        Rotation,
        InverseMass,
        InverseInertia,
        Force,
        Damping,
        CircleCollider,
        BoxCollider,
        CollisionMaterial,
        CollisionFilter,
        Static,
        Kinematic,
        Dynamic,
        Sensor,
        CollisionEvents
    );

    ecs_with(Static, Position, Rotation);
    ecs_with(Kinematic, Position, Rotation, Velocity, AngularVelocity);
    ecs_with(Dynamic, Position, Rotation, Velocity, AngularVelocity, InverseMass, InverseInertia);

    ecs_with(
        CircleCollider,
        Position,
        CollisionMaterial,
        CollisionFilter
    );
    ecs_with(
        BoxCollider,
        Position,
        Rotation,
        CollisionMaterial,
        CollisionFilter
    );

    ECS_RESOURCE_REGISTER(SipSettings);
    SipSettings settings = props->use_custom_settings ? props->settings
                                                      : (SipSettings){
                                                            .gravity_x = 0.0f,
                                                            .gravity_y = -9.81f,
                                                            .fixed_dt = 1.0f / 60.0f,
                                                            .max_frame_dt = 0.25f,
                                                            .max_substeps = 8,
                                                            .solver_iterations = 6,
                                                            .restitution_threshold = 1.0f,
                                                            .penetration_slop = 0.005f,
                                                            .penetration_correction = 0.8f,
                                                        };
    if (!(settings.fixed_dt > 0.0f)) {
        settings.fixed_dt = 1.0f / 60.0f;
    }

    if (!(settings.max_frame_dt > 0.0f)) {
        settings.max_frame_dt = 0.25f;
    }

    if (settings.max_substeps == 0) {
        settings.max_substeps = 1;
    }

    if (settings.solver_iterations == 0) {
        settings.solver_iterations = 1;
    }

    if (!(settings.restitution_threshold >= 0.0f)) {
        settings.restitution_threshold = 1.0f;
    }

    ecs_set_resource_rid(ecs_id(SipSettings), &settings);

    ECS_RESOURCE_REGISTER(SipPhysicsRuntime);
    ecs_set_resource(SipPhysicsRuntime, {
        .accumulator = 0.0,
        .tick = 0,
        .gravity_query = 0,
        .velocity_query = 0,
        .angular_velocity_query = 0,
    });

    ECS_RESOURCE_REGISTER(SipCollisionStats);
    ecs_set_resource(SipCollisionStats, {
        .proxy_count = 0,
        .candidate_count = 0,
        .contact_count = 0,
        .contact_cache_count = 0,
        .contact_cache_hit_count = 0,
        .event_pair_count = 0,
        .event_dispatch_count = 0,
        .scratch_growth_count = 0,
    });
    ECS_RESOURCE_REGISTER(SipCollisionCapacity);
    ecs_set_resource(SipCollisionCapacity, {
        .proxy_capacity = 256,
        .pair_capacity = 512,
        .contact_capacity = 256,
        .event_pair_capacity = 128,
    });
    ECS_RESOURCE_REGISTER(SipCollisionRuntime);
    SipCollisionRuntime runtime = {0};
    ecs_set_resource_rid(ecs_id(SipCollisionRuntime), &runtime);

    ecs_event_register(&SipCollisionEnter);
    ecs_event_register(&SipCollisionStay);
    ecs_event_register(&SipCollisionExit);

    siphysics_phase = ecs_phase_init(&(ecs_phase_desc_t){
        .name = "siphysics",
        .after = EcsPreUpdate,
        .before = EcsOnUpdate,
    });

    siphysics_collision_init();
    siphysics_register_systems();
}
