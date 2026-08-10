#include <siphysics.h>

static void on_collision_enter(ecs_observer_event_t *event) {
    const SipCollisionEvent *collision =
        static_cast<const SipCollisionEvent *>(event->trigger_data);
    (void)collision;
}

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    ecs::init();

    ecs::import<siphysics>();
    ecs_observer_desc_t observer_desc{};
    observer_desc.on = SipCollisionEnter;
    observer_desc.query.terms[0] = ecs_in(CollisionEvents);
    observer_desc.callback = on_collision_enter;
    ecs_observer_init(&observer_desc);

    ecs::entity ball = ecs::entity::create();
    ball.add<Dynamic>();
    ball.add<CollisionEvents>();
    ball.set(Position{0.0f, 1.0f});
    ball.set(Velocity{0.0f, 0.0f});
    ball.set(InverseMass{1.0f});
    ball.set(CircleCollider{0.5f});
    ball.set(CollisionMaterial{0.5f, 0.0f});
    ball.set(CollisionFilter{1u, UINT32_MAX});

    ecs::entity box = ecs::entity::create();
    box.add<Static>();
    box.add<BoxCollider>();
    box.set(Position{0.0f, 0.0f});
    box.set(Rotation{0.0f});
    box.set(CollisionMaterial{0.5f, 0.0f});
    box.set(CollisionFilter{1u, UINT32_MAX});

    ecs::fini();
}
