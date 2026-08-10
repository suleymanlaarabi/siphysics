#ifndef SIPHYSICS_COMPONENTS_H
#define SIPHYSICS_COMPONENTS_H

#include "siphysics/bake_config.h"
#include <siecs.h>

#ifdef __cplusplus
extern "C" {
#endif

ECS_COMPONENT_DECLARE_CPP(
    Position,
    ECS_CPP_FIELDS(float x; float y;),
    ECS_CPP_METHODS()
);

ECS_COMPONENT_DECLARE_CPP(
    Velocity,
    ECS_CPP_FIELDS(float x; float y;),
    ECS_CPP_METHODS()
);

ECS_COMPONENT_DECLARE_CPP(
    AngularVelocity,
    ECS_CPP_FIELDS(float value;),
    ECS_CPP_METHODS()
);

ECS_COMPONENT_DECLARE_CPP(
    Rotation,
    ECS_CPP_FIELDS(float angle;),
    ECS_CPP_METHODS()
);

ECS_COMPONENT_DECLARE_CPP(
    InverseMass,
    ECS_CPP_FIELDS(float value;),
    ECS_CPP_METHODS()
);

ECS_COMPONENT_DECLARE_CPP(
    Force,
    ECS_CPP_FIELDS(float x; float y;),
    ECS_CPP_METHODS()
);

ECS_COMPONENT_DECLARE_CPP(
    Damping,
    ECS_CPP_FIELDS(float linear; float angular;),
    ECS_CPP_METHODS()
);

ECS_TAG_DECLARE(Static);
ECS_TAG_DECLARE(Kinematic);
ECS_TAG_DECLARE(Dynamic);

#ifdef __cplusplus
}
#endif

#endif
