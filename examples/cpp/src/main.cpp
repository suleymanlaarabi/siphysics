#include <siphysics.h>


int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    ecs::init();

    ecs::import<siphysics>();
    ecs::entity entity = ecs::entity::create();
    entity.add<Dynamic>();
    entity.set(Position{});
    entity.set(Velocity{});
    entity.set(Rotation{});

    ecs::fini();
}
