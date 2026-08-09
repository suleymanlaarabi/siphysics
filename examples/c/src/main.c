#include <siphysics.h>

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    ecs_init();

    ECS_MODULE_IMPORT(siphysics, { .use_custom_settings = false });
    ecs_entity_t entity = ecs_new();
    ecs_add(entity, Dynamic);

    ecs_fini();
}
