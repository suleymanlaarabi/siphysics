#include "internal.h"
#include "siphysics/physics.h"

ECS_RESOURCE_DEFINE(SipSettings);
ECS_MODULE_DEFINE(siphysics);

ecs_phase_t siphysics_phase = ECS_PHASE_NONE;

void siphysics_import(const siphysics_props_t *props) {
    ECS_COMPONENT_REGISTER(
        Position,
        Velocity,
        Rotation,
        InverseMass,
        Force,
        Damping,
        Static,
        Kinematic,
        Dynamic
    );

    ecs_with(Static, Position, Rotation);
    ecs_with(Kinematic, Position, Rotation, Velocity);
    ecs_with(Dynamic, Position, Rotation, Velocity, InverseMass);

    ECS_RESOURCE_REGISTER(SipSettings);
    SipSettings settings = props->use_custom_settings ? props->settings
                                                      : (SipSettings){
                                                            .gravity_x = 0.0f,
                                                            .gravity_y = -9.81f,
                                                            .fixed_dt = 1.0f / 60.0f,
                                                            .max_frame_dt = 0.25f,
                                                            .max_substeps = 8,
                                                        };
    ecs_set_resource_rid(ecs_id(SipSettings), &settings);

    siphysics_phase = ecs_phase_init(&(ecs_phase_desc_t){
        .name = "siphysics",
        .after = EcsPreUpdate,
        .before = EcsOnUpdate,
    });

    siphysics_register_systems();
}
