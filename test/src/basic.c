#include <test.h>

void basic_simple(void) {
    ecs_init();
    ECS_MODULE_IMPORT(siphysics, { .use_custom_settings = false });
    test_str("siphysics", ecs_module_name(ecs_id(siphysics)));
    ecs_fini();
}

void basic_module_and_defaults(void) {
    ecs_init();
    ECS_MODULE_IMPORT(
        siphysics,
        {
            .use_custom_settings = true,
            .settings = {
                .gravity_x = 1.0f,
                .gravity_y = 2.0f,
                .fixed_dt = 0.5f,
                .max_frame_dt = 3.0f,
                .max_substeps = 4,
                .solver_iterations = 3,
                .penetration_slop = 0.01f,
                .penetration_correction = 0.7f,
            },
        }
    );

    const SipSettings *settings = ecs_get_resource_read(SipSettings);
    test_assert(settings->gravity_x == 1.0f);
    test_assert(settings->gravity_y == 2.0f);
    test_assert(settings->fixed_dt == 0.5f);
    test_assert(settings->max_frame_dt == 3.0f);
    test_int(4, settings->max_substeps);
    test_int(3, settings->solver_iterations);
    test_assert(settings->penetration_slop == 0.01f);
    test_assert(settings->penetration_correction == 0.7f);
    ecs_fini();

    ecs_init();
    ECS_MODULE_IMPORT(siphysics, { .use_custom_settings = false });
    settings = ecs_get_resource_read(SipSettings);
    test_assert(settings->gravity_x == 0.0f);
    test_assert(settings->gravity_y == -9.81f);
    test_assert(settings->fixed_dt == 1.0f / 60.0f);
    test_assert(settings->max_frame_dt == 0.25f);
    test_int(8, settings->max_substeps);
    test_int(6, settings->solver_iterations);
    test_assert(settings->penetration_slop == 0.005f);
    test_assert(settings->penetration_correction == 0.8f);
    ecs_fini();

    ecs_init();
    ECS_MODULE_IMPORT(
        siphysics,
        {
            .use_custom_settings = true,
            .settings = {
                .gravity_x = 0.0f,
                .gravity_y = 0.0f,
                .fixed_dt = 0.0f,
                .max_frame_dt = 0.0f,
                .max_substeps = 0,
                .solver_iterations = 0,
                .penetration_slop = 0.005f,
                .penetration_correction = 0.8f,
            },
        }
    );
    settings = ecs_get_resource_read(SipSettings);
    test_assert(settings->fixed_dt == 1.0f / 60.0f);
    test_assert(settings->max_frame_dt == 0.25f);
    test_int(1, settings->max_substeps);
    test_int(1, settings->solver_iterations);
    ecs_fini();
}

void basic_body_components(void) {
    ecs_init();
    ECS_MODULE_IMPORT(siphysics, { .use_custom_settings = false });

    ecs_entity_t static_body = ecs_new();
    ecs_add(static_body, Static);
    test_true(ecs_has(static_body, Static));
    test_true(ecs_has(static_body, Position));
    test_true(ecs_has(static_body, Rotation));
    test_false(ecs_has(static_body, Velocity));

    ecs_entity_t kinematic_body = ecs_new();
    ecs_add(kinematic_body, Kinematic);
    test_true(ecs_has(kinematic_body, Position));
    test_true(ecs_has(kinematic_body, Rotation));
    test_true(ecs_has(kinematic_body, Velocity));
    test_false(ecs_has(kinematic_body, InverseMass));

    ecs_entity_t dynamic_body = ecs_new();
    ecs_add(dynamic_body, Dynamic);
    test_true(ecs_has(dynamic_body, Position));
    test_true(ecs_has(dynamic_body, Rotation));
    test_true(ecs_has(dynamic_body, Velocity));
    test_true(ecs_has(dynamic_body, InverseMass));
    test_assert(ecs_get(dynamic_body, InverseMass)->value == 1.0f);
    test_false(ecs_has(dynamic_body, Force));
    test_false(ecs_has(dynamic_body, Damping));

    ecs_get(dynamic_body, Position)->x = 12.0f;
    ecs_get(dynamic_body, Rotation)->angle = 0.5f;
    ecs_get(dynamic_body, Velocity)->x = 3.0f;
    ecs_set(dynamic_body, Damping, { .linear = 2.0f, .angular = 4.0f });
    ecs_set(dynamic_body, InverseMass, { .value = 0.25f });

    ecs_defer_begin();
    ecs_remove(dynamic_body, Dynamic);
    ecs_add(dynamic_body, Kinematic);
    ecs_remove(dynamic_body, InverseMass);
    ecs_remove(dynamic_body, Damping);
    ecs_defer_end();

    test_true(ecs_has(dynamic_body, Kinematic));
    test_false(ecs_has(dynamic_body, Dynamic));
    test_true(ecs_has(dynamic_body, Position));
    test_true(ecs_has(dynamic_body, Rotation));
    test_true(ecs_has(dynamic_body, Velocity));
    test_false(ecs_has(dynamic_body, InverseMass));
    test_false(ecs_has(dynamic_body, Force));
    test_false(ecs_has(dynamic_body, Damping));
    test_assert(ecs_get(dynamic_body, Position)->x == 12.0f);
    test_assert(ecs_get(dynamic_body, Rotation)->angle == 0.5f);
    test_assert(ecs_get(dynamic_body, Velocity)->x == 3.0f);

    ecs_defer_begin();
    ecs_remove(dynamic_body, Kinematic);
    ecs_add(dynamic_body, Dynamic);
    ecs_set(dynamic_body, InverseMass, { .value = 0.5f });
    ecs_defer_end();
    test_true(ecs_has(dynamic_body, Dynamic));
    test_false(ecs_has(dynamic_body, Kinematic));
    test_assert(ecs_get(dynamic_body, InverseMass)->value == 0.5f);

    ecs_remove(dynamic_body, Dynamic);
    ecs_remove(dynamic_body, Position);
    ecs_remove(dynamic_body, Rotation);
    ecs_remove(dynamic_body, Velocity);
    ecs_remove(dynamic_body, InverseMass);
    ecs_remove(dynamic_body, Force);
    ecs_remove(dynamic_body, Damping);
    test_true(ecs_is_alive(dynamic_body));

    ecs_kill(dynamic_body);
    test_false(ecs_is_alive(dynamic_body));
    ecs_fini();
}
