#include "siphysics/components.h"

ECS_CTOR(InverseMass, { .value = 1.0f });

ECS_COMPONENT_DEFINE(Position);
ECS_COMPONENT_DEFINE(Velocity);
ECS_COMPONENT_DEFINE(AngularVelocity);
ECS_COMPONENT_DEFINE(Rotation);
ECS_COMPONENT_DEFINE(InverseMass, .ops.ctor = ecs_ctor_id(InverseMass));
ECS_COMPONENT_DEFINE(Force);
ECS_COMPONENT_DEFINE(Damping);

ECS_TAG_DEFINE(Static);
ECS_TAG_DEFINE(Kinematic);
ECS_TAG_DEFINE(Dynamic);
