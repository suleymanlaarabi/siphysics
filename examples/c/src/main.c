#include <siphysics.h>

static void on_collision_enter(ecs_observer_event_t *event) {
    const SipCollisionEvent *collision = event->trigger_data;
    (void)collision;
}

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    ecs_init();

    ECS_MODULE_IMPORT(siphysics, { .use_custom_settings = false });
    ecs_observer({
        .on = SipCollisionEnter,
        .query.terms = { ecs_in(CollisionEvents) },
        .callback = on_collision_enter,
    });

    ecs_entity_t ball = ecs_new();
    ecs_add(ball, Dynamic);
    ecs_add(ball, CollisionEvents);
    ecs_set(ball, Position, { .x = 0.0f, .y = 1.0f });
    ecs_set(ball, Velocity, { .x = 0.0f, .y = 0.0f });
    ecs_set(ball, InverseMass, { .value = 1.0f });
    ecs_set(ball, CircleCollider, { .radius = 0.5f });
    ecs_set(ball, CollisionMaterial, { .friction = 0.5f, .restitution = 0.0f });
    ecs_set(ball, CollisionFilter, { .layer = 1u, .mask = UINT32_MAX });

    ecs_entity_t box = ecs_new();
    ecs_add(box, Static);
    ecs_add(box, BoxCollider);
    ecs_set(box, Position, { .x = 0.0f, .y = 0.0f });
    ecs_set(box, Rotation, { .angle = 0.0f });
    ecs_set(box, CollisionMaterial, { .friction = 0.5f, .restitution = 0.0f });
    ecs_set(box, CollisionFilter, { .layer = 1u, .mask = UINT32_MAX });

    ecs_fini();
}
