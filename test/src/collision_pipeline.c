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
        .restitution_threshold = 1.0f,
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
        .restitution_threshold = 1.0f,
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
        .restitution_threshold = 1.0f,
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
        .restitution_threshold = 1.0f,
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
        .restitution_threshold = 1.0f,
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
        .restitution_threshold = 1.0f,
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

static void collision_regression_import(float gravity_y, uint32_t iterations,
                                        float slop, float correction) {
    collision_import_settings((SipSettings){
        .gravity_x = 0.0f, .gravity_y = gravity_y,
        .fixed_dt = 1.0f / 60.0f, .max_frame_dt = 0.25f,
        .max_substeps = 8, .solver_iterations = iterations,
        .restitution_threshold = 1.0f, .penetration_slop = slop,
        .penetration_correction = correction,
    });
}

static void collision_regression_material(ecs_entity_t entity, float friction, float restitution) {
    ecs_set(entity, CollisionMaterial, { .friction = friction, .restitution = restitution });
}

static ecs_entity_t collision_regression_ground(float y, float friction, float restitution) {
    ecs_entity_t ground = collision_static_box(0.0f, y, 0.0f);
    ecs_set(ground, BoxCollider, { .half_width = 20.0f, .half_height = 0.5f });
    collision_regression_material(ground, friction, restitution);
    return ground;
}

void collision_pipeline_position_correction_once(void) {
    collision_regression_import(0.0f, 8, 0.0f, 0.5f);
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 0.75f);
    ecs_set(circle, CircleCollider, { .radius = 0.5f });
    collision_static_box(0.0f, 0.0f, 0.0f);
    collision_step();
    test_assert(ecs_get(circle, Position)->y > 0.8749f);
    test_assert(ecs_get(circle, Position)->y < 0.8751f);
    ecs_fini();
}

void collision_pipeline_resting_circle(void) {
    collision_regression_import(-10.0f, 8, 0.005f, 0.8f);
    collision_regression_ground(0.0f, 0.5f, 0.0f);
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 5.0f);
    ecs_set(circle, CircleCollider, { .radius = 0.5f });
    collision_regression_material(circle, 0.5f, 0.0f);
    for (uint32_t i = 0; i < 600; i++) collision_step();
    test_assert(ecs_get(circle, Position)->y > 0.98f && ecs_get(circle, Position)->y < 1.02f);
    test_assert(ecs_get(circle, Velocity)->y > -0.1f && ecs_get(circle, Velocity)->y < 0.1f);
    ecs_fini();
}

static float collision_regression_bounce(float velocity_y) {
    collision_regression_import(0.0f, 8, 0.0f, 0.8f);
    collision_regression_ground(0.0f, 0.0f, 1.0f);
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 0.99f);
    ecs_set(circle, CircleCollider, { .radius = 0.5f });
    ecs_set(circle, Velocity, { .x = 0.0f, .y = velocity_y });
    collision_regression_material(circle, 0.0f, 1.0f);
    collision_step();
    float result = ecs_get(circle, Velocity)->y;
    ecs_fini();
    return result;
}

void collision_pipeline_restitution_threshold(void) {
    test_assert(collision_regression_bounce(-0.2f) <= 0.05f);
    test_assert(collision_regression_bounce(-4.0f) > 1.0f);
}

void collision_pipeline_friction_accumulation(void) {
    collision_regression_import(0.0f, 8, 0.0f, 0.8f);
    collision_regression_ground(0.0f, 1.0f, 0.0f);
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 0.99f);
    ecs_set(circle, CircleCollider, { .radius = 0.5f });
    ecs_set(circle, Velocity, { .x = 5.0f, .y = -1.0f });
    collision_regression_material(circle, 1.0f, 0.0f);
    collision_step();
    test_assert(ecs_get(circle, Velocity)->x < 5.0f);
    test_assert(ecs_get(circle, Velocity)->y >= -0.05f);
    ecs_fini();
}

static float collision_regression_position(uint32_t iterations) {
    collision_regression_import(0.0f, iterations, 0.0f, 0.8f);
    ecs_entity_t circle = collision_dynamic_circle(0.0f, 0.75f);
    collision_static_box(0.0f, 0.0f, 0.0f);
    collision_step();
    float y = ecs_get(circle, Position)->y;
    ecs_fini();
    return y;
}

void collision_pipeline_solver_iterations_position_independent(void) {
    const float y_a = collision_regression_position(1);
    const float y_b = collision_regression_position(16);
    test_assert(y_a - y_b < 0.0001f && y_b - y_a < 0.0001f);
}

void collision_pipeline_many_falling_circles(void) {
    collision_regression_import(-9.81f, 8, 0.005f, 0.8f);
    collision_regression_ground(-0.5f, 0.5f, 0.0f);
    ecs_entity_t circles[16];
    uint32_t count = 0;
    for (int y = 0; y < 4; y++) for (int x = 0; x < 4; x++) {
        ecs_entity_t circle = collision_dynamic_circle(-3.0f + 2.0f * x, 2.0f + 2.0f * y);
        circles[count++] = circle;
        ecs_set(circle, CircleCollider, { .radius = 0.5f });
        collision_regression_material(circle, 0.5f, 0.0f);
    }
    for (uint32_t i = 0; i < 600; i++) collision_step();
    for (uint32_t i = 0; i < count; i++) {
        const Position *position = ecs_get(circles[i], Position);
        const Velocity *velocity = ecs_get(circles[i], Velocity);
        test_assert(position->x == position->x && position->y == position->y);
        test_assert(velocity->x == velocity->x && velocity->y == velocity->y);
        test_assert(position->y > 0.45f && position->y < 10.0f);
        test_assert(velocity->y > -0.5f && velocity->y < 0.5f);
    }
    ecs_fini();
}
