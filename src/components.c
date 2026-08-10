#include "siphysics/components.h"

ECS_CTOR(InverseMass, { .value = 1.0f });
ECS_CTOR(CircleCollider, { .radius = 0.5f });
ECS_CTOR(BoxCollider, { .half_width = 0.5f, .half_height = 0.5f });
ECS_CTOR(CollisionMaterial, { .friction = 0.5f, .restitution = 0.0f });
ECS_CTOR(CollisionFilter, { .layer = 1u, .mask = UINT32_MAX });

ECS_COMPONENT_DEFINE(Position);
ECS_COMPONENT_DEFINE(Velocity);
ECS_COMPONENT_DEFINE(AngularVelocity);
ECS_COMPONENT_DEFINE(Rotation);
ECS_COMPONENT_DEFINE(InverseMass, .ops.ctor = ecs_ctor_id(InverseMass));
ECS_COMPONENT_DEFINE(Force);
ECS_COMPONENT_DEFINE(Damping);
ECS_COMPONENT_DEFINE(CircleCollider, .ops.ctor = ecs_ctor_id(CircleCollider));
ECS_COMPONENT_DEFINE(BoxCollider, .ops.ctor = ecs_ctor_id(BoxCollider));
ECS_COMPONENT_DEFINE(CollisionMaterial, .ops.ctor = ecs_ctor_id(CollisionMaterial));
ECS_COMPONENT_DEFINE(CollisionFilter, .ops.ctor = ecs_ctor_id(CollisionFilter));

ECS_TAG_DEFINE(Static);
ECS_TAG_DEFINE(Kinematic);
ECS_TAG_DEFINE(Dynamic);
ECS_TAG_DEFINE(Sensor);
ECS_TAG_DEFINE(CollisionEvents);
