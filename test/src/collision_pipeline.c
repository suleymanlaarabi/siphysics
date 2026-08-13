#include <test.h>

static void collision_import_settings(SipSettings settings) {
    ecs_init();
    ECS_MODULE_IMPORT(
        siphysics,
        {
            .use_custom_settings = true,
            .settings = settings,
        }
    );
}

static void collision_import(float gravity_x, float gravity_y) {
    SipSettings settings = {
        .gravity_x = gravity_x,
        .gravity_y = gravity_y,
        .fixed_dt = 1.0f / 60.0f,
        .max_frame_dt = 0.25f,
        .max_substeps = 8,
        .solver_iterations = 8,
        .penetration_slop = 0.001f,
        .penetration_correction = 1.0f,
    };
    collision_import_settings(settings);
}

static void collision_step(void) {
    siphysics_advance(1.0f / 60.0f);
}

static ecs_entity_t collision_dynamic_circle(float x, float y) {
    ecs_entity_t entity = ecs_new();
    ecs_add(entity, Dynamic);
    ecs_add(entity, CircleCollider);
    ecs_set(entity, Position, { .x = x, .y = y });
    return entity;
}

static ecs_entity_t collision_static_circle(float x, float y) {
    ecs_entity_t entity = ecs_new();
    ecs_add(entity, Static);
    ecs_add(entity, CircleCollider);
    ecs_set(entity, Position, { .x = x, .y = y });
    return entity;
}

static ecs_entity_t collision_dynamic_box(float x, float y, float angle) {
    ecs_entity_t entity = ecs_new();
    ecs_add(entity, Dynamic);
    ecs_add(entity, BoxCollider);
    ecs_set(entity, Position, { .x = x, .y = y });
    ecs_set(entity, Rotation, { .angle = angle });
    return entity;
}

static ecs_entity_t collision_static_box(float x, float y, float angle) {
    ecs_entity_t entity = ecs_new();
    ecs_add(entity, Static);
    ecs_add(entity, BoxCollider);
    ecs_set(entity, Position, { .x = x, .y = y });
    ecs_set(entity, Rotation, { .angle = angle });
    return entity;
}

void collision_pipeline_circle_box(void) {
    collision_import(0.0f, 0.0f);
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 0.75f);
    collision_static_box(0.0f, 0.0f, 0.0f);
    collision_step();
    test_assert(ecs_get(circle, Position)->y > 0.75f);
    test_assert(ecs_get_resource_read(SipCollisionStats)->candidate_count == 1);
    test_assert(ecs_get_resource_read(SipCollisionStats)->contact_count == 1);
    ecs_fini();
}

void collision_pipeline_all_shapes(void) {
    collision_import(0.0f, 0.0f);
    ecs_entity_t circle_circle = collision_dynamic_circle(0.75f, 0.0f);
    collision_static_circle(0.0f, 0.0f);
    ecs_entity_t box_circle = collision_dynamic_box(0.0f, 0.75f, 0.78539816f);
    collision_static_circle(0.0f, 0.0f);
    ecs_entity_t box_box = collision_dynamic_box(0.75f, 0.0f, 0.78539816f);
    collision_static_box(0.0f, 0.0f, -0.35f);
    collision_step();
    test_assert(ecs_get(circle_circle, Position)->x > 0.75f);
    test_assert(ecs_get(box_circle, Position)->x != 0.0f ||
                ecs_get(box_circle, Position)->y != 0.75f);
    test_assert(ecs_get(box_box, Position)->x > 0.75f || ecs_get(box_box, Position)->y > 0.0f);
    test_assert(ecs_get_resource_read(SipCollisionStats)->contact_count >= 3);
    ecs_fini();
}

void collision_pipeline_sensor_filter(void) {
    collision_import(0.0f, 0.0f);
    ecs_entity_t sensor_circle = collision_static_circle(3.0f, 0.0f);
    ecs_add(sensor_circle, Sensor);
    collision_static_circle(0.0f, 0.0f);
    ecs_entity_t filtered_circle = collision_dynamic_circle(0.75f, 0.0f);
    ecs_set(filtered_circle, CollisionFilter, { .layer = 2u, .mask = 2u });
    ecs_entity_t sensor_test = collision_dynamic_circle(0.0f, 0.0f);
    collision_step();
    test_assert(ecs_get(sensor_test, Position)->x != 0.0f ||
                ecs_get(sensor_test, Position)->y != 0.0f);
    test_assert(ecs_get(filtered_circle, Position)->x == 0.75f);
    test_assert(ecs_get_resource_read(SipCollisionStats)->contact_count >= 1);
    ecs_fini();
}

void collision_pipeline_invalid_archetypes(void) {
    collision_import(0.0f, 0.0f);
    ecs_entity_t invalid_shape = collision_dynamic_circle(0.0f, 0.0f);
    ecs_add(invalid_shape, BoxCollider);
    ecs_entity_t invalid_body = collision_static_circle(0.75f, 0.0f);
    ecs_add(invalid_body, Dynamic);
    collision_step();
    test_assert(ecs_get_resource_read(SipCollisionStats)->proxy_count == 0);
    test_assert(ecs_get(invalid_shape, Position)->x == 0.0f);
    ecs_fini();
}

void collision_pipeline_dynamic_kinematic_masses(void) {
    collision_import(0.0f, 0.0f);
    ecs_entity_t dynamic = collision_dynamic_circle(0.75f, 0.0f);
    ecs_entity_t kinematic = ecs_new();
    ecs_add(kinematic, Kinematic);
    ecs_add(kinematic, CircleCollider);
    ecs_set(kinematic, Position, { .x = 0.0f, .y = 0.0f });
    ecs_set(kinematic, Velocity, { .x = 0.0f, .y = 0.0f });
    collision_step();
    test_assert(ecs_get(dynamic, Position)->x != 0.75f);
    test_assert(ecs_get(kinematic, Position)->x == 0.0f);
    test_assert(ecs_get(kinematic, Velocity)->x == 0.0f);
    ecs_fini();

    collision_import(0.0f, 0.0f);
    dynamic = collision_dynamic_circle(0.75f, 0.0f);
    ecs_set(dynamic, InverseMass, { .value = 0.5f });
    ecs_entity_t second_dynamic = collision_dynamic_circle(0.0f, 0.0f);
    ecs_set(second_dynamic, InverseMass, { .value = 1.0f });
    collision_step();
    test_assert(ecs_get(dynamic, Position)->x != 0.75f);
    test_assert(ecs_get(second_dynamic, Position)->x != 0.0f);
    ecs_fini();
}

void collision_pipeline_fixed_timestep_accumulator(void) {
    collision_import_settings((SipSettings){
        .gravity_x = 0.0f,
        .gravity_y = -8.0f,
        .fixed_dt = 0.125f,
        .max_frame_dt = 1.0f,
        .max_substeps = 8,
        .solver_iterations = 8,
        .penetration_slop = 0.001f,
        .penetration_correction = 1.0f,
    });
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 10.0f);
    siphysics_advance(0.0625f);
    test_assert(ecs_get(circle, Velocity)->y == 0.0f);
    test_assert(ecs_get(circle, Position)->y == 10.0f);

    siphysics_advance(0.0625f);
    test_assert(ecs_get(circle, Velocity)->y == -1.0f);
    test_assert(ecs_get(circle, Position)->y == 9.875f);
    ecs_fini();
}

typedef struct FixedTimestepState {
    Position position;
    Velocity velocity;
} FixedTimestepState;

static FixedTimestepState collision_pipeline_run_split(
    uint32_t calls,
    float frame_dt
) {
    collision_import_settings((SipSettings){
        .gravity_x = 0.0f,
        .gravity_y = -8.0f,
        .fixed_dt = 1.0f / 64.0f,
        .max_frame_dt = 1.0f,
        .max_substeps = 8,
        .solver_iterations = 8,
        .penetration_slop = 0.001f,
        .penetration_correction = 1.0f,
    });
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 10.0f);
    ecs_set(circle, Velocity, { .x = 1.0f, .y = 0.0f });

    for (uint32_t i = 0; i < calls; i++) {
        siphysics_advance(frame_dt);
    }

    FixedTimestepState state = {
        .position = *ecs_get(circle, Position),
        .velocity = *ecs_get(circle, Velocity),
    };
    ecs_fini();
    return state;
}

void collision_pipeline_fixed_timestep_split(void) {
    const FixedTimestepState a =
        collision_pipeline_run_split(64, 1.0f / 64.0f);
    const FixedTimestepState b =
        collision_pipeline_run_split(128, 1.0f / 128.0f);
    const FixedTimestepState c =
        collision_pipeline_run_split(32, 1.0f / 32.0f);

    test_assert(a.position.x == b.position.x);
    test_assert(a.position.x == c.position.x);
    test_assert(a.position.y == b.position.y);
    test_assert(a.position.y == c.position.y);
    test_assert(a.velocity.x == b.velocity.x);
    test_assert(a.velocity.x == c.velocity.x);
    test_assert(a.velocity.y == b.velocity.y);
    test_assert(a.velocity.y == c.velocity.y);
}

void collision_pipeline_fixed_timestep_max_substeps(void) {
    collision_import_settings((SipSettings){
        .gravity_x = 0.0f,
        .gravity_y = 0.0f,
        .fixed_dt = 0.125f,
        .max_frame_dt = 1.0f,
        .max_substeps = 2,
        .solver_iterations = 8,
        .penetration_slop = 0.001f,
        .penetration_correction = 1.0f,
    });
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 0.0f);
    ecs_set(circle, Velocity, { .x = 1.0f, .y = 0.0f });

    siphysics_advance(0.5f);
    test_assert(ecs_get(circle, Position)->x == 0.25f);

    siphysics_advance(0.125f);
    test_assert(ecs_get(circle, Position)->x == 0.375f);
    ecs_fini();
}

void collision_pipeline_fixed_timestep_clamp(void) {
    collision_import_settings((SipSettings){
        .gravity_x = 0.0f,
        .gravity_y = 0.0f,
        .fixed_dt = 0.125f,
        .max_frame_dt = 0.25f,
        .max_substeps = 8,
        .solver_iterations = 8,
        .penetration_slop = 0.001f,
        .penetration_correction = 1.0f,
    });
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 0.0f);
    ecs_set(circle, Velocity, { .x = 1.0f, .y = 0.0f });

    siphysics_advance(10.0f);
    test_assert(ecs_get(circle, Position)->x == 0.25f);
    ecs_fini();
}

void collision_pipeline_fixed_timestep_angular(void) {
    collision_import_settings((SipSettings){
        .gravity_x = 0.0f,
        .gravity_y = 0.0f,
        .fixed_dt = 0.125f,
        .max_frame_dt = 1.0f,
        .max_substeps = 8,
        .solver_iterations = 8,
        .penetration_slop = 0.001f,
        .penetration_correction = 1.0f,
    });
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 0.0f);
    ecs_set(circle, Rotation, { .angle = 0.0f });
    ecs_set(circle, AngularVelocity, { .value = 2.0f });

    siphysics_advance(0.25f);
    test_assert(ecs_get(circle, Rotation)->angle == 0.5f);
    ecs_fini();
}
