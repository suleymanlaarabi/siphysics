#ifndef SIPHYSICS_PHYSICS_H
#define SIPHYSICS_PHYSICS_H

#include "siphysics/components.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

ECS_RESOURCE_DECLARE_CPP(
    SipSettings,
    ECS_CPP_FIELDS(
        float gravity_x;
        float gravity_y;
        float fixed_dt;
        float max_frame_dt;
        uint32_t max_substeps;
        uint32_t solver_iterations;
        float penetration_slop;
        float penetration_correction;
    ),
    ECS_CPP_METHODS(
        SipSettings()
            : gravity_x(0), gravity_y(-9.81f), fixed_dt(1.0f / 60.0f), max_frame_dt(0.25f),
              max_substeps(8), solver_iterations(6), penetration_slop(0.005f),
              penetration_correction(0.8f) {}
    )
);

ECS_RESOURCE_DECLARE_CPP(
    SipCollisionStats,
    ECS_CPP_FIELDS(
        uint32_t proxy_count;
        uint32_t candidate_count;
        uint32_t contact_count;
        uint64_t scratch_growth_count;
    ),
    ECS_CPP_METHODS()
);

ECS_MODULE_DECLARE(siphysics, {
    bool use_custom_settings;
    SipSettings settings;
});

extern ecs_phase_t siphysics_phase;

#ifdef __cplusplus
}
#endif

#endif
