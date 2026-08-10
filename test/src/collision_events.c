#include <test.h>

static uint32_t sip_enter_count;
static uint32_t sip_stay_count;
static uint32_t sip_exit_count;
static SipCollisionEvent sip_last_enter;
static SipCollisionEvent sip_last_stay;
static SipCollisionEvent sip_last_exit;

static void sip_collision_event_callback(ecs_observer_event_t *event) {
    const SipCollisionEvent *payload = event->trigger_data;
    if (event->event == SipCollisionEnter) {
        sip_enter_count++;
        sip_last_enter = *payload;
    } else if (event->event == SipCollisionStay) {
        sip_stay_count++;
        sip_last_stay = *payload;
    } else if (event->event == SipCollisionExit) {
        sip_exit_count++;
        sip_last_exit = *payload;
    }
}

static void sip_collision_events_import(void) {
    ecs_init();
    ECS_MODULE_IMPORT(
        siphysics,
        {
            .use_custom_settings = true,
            .settings = {
                .gravity_x = 0.0f,
                .gravity_y = 0.0f,
                .fixed_dt = 1.0f / 60.0f,
                .max_frame_dt = 0.25f,
                .max_substeps = 8,
                .solver_iterations = 6,
                .penetration_slop = 0.001f,
                .penetration_correction = 1.0f,
            },
        }
    );
    ecs_observer({
        .on = SipCollisionEnter,
        .query.terms = { ecs_in(CollisionEvents) },
        .callback = sip_collision_event_callback,
    });
    ecs_observer({
        .on = SipCollisionStay,
        .query.terms = { ecs_in(CollisionEvents) },
        .callback = sip_collision_event_callback,
    });
    ecs_observer({
        .on = SipCollisionExit,
        .query.terms = { ecs_in(CollisionEvents) },
        .callback = sip_collision_event_callback,
    });
    sip_enter_count = 0;
    sip_stay_count = 0;
    sip_exit_count = 0;
    sip_last_enter = (SipCollisionEvent){0};
    sip_last_stay = (SipCollisionEvent){0};
    sip_last_exit = (SipCollisionEvent){0};
}

static ecs_entity_t sip_event_circle(float x, float y, bool events, bool sensor) {
    ecs_entity_t entity = ecs_new();
    ecs_add(entity, Kinematic);
    ecs_add(entity, CircleCollider);
    if (events) {
        ecs_add(entity, CollisionEvents);
    }
    if (sensor) {
        ecs_add(entity, Sensor);
    }
    ecs_set(entity, Position, { .x = x, .y = y });
    return entity;
}

static ecs_entity_t sip_event_static_circle(float x, float y, bool events, bool sensor) {
    ecs_entity_t entity = ecs_new();
    ecs_add(entity, Static);
    ecs_add(entity, CircleCollider);
    if (events) {
        ecs_add(entity, CollisionEvents);
    }
    if (sensor) {
        ecs_add(entity, Sensor);
    }
    ecs_set(entity, Position, { .x = x, .y = y });
    return entity;
}

static ecs_entity_t sip_event_static_box(float x, float y, bool events, bool sensor) {
    ecs_entity_t entity = ecs_new();
    ecs_add(entity, Static);
    ecs_add(entity, BoxCollider);
    if (events) {
        ecs_add(entity, CollisionEvents);
    }
    if (sensor) {
        ecs_add(entity, Sensor);
    }
    ecs_set(entity, Position, { .x = x, .y = y });
    ecs_set(entity, Rotation, { .angle = 0.35f });
    return entity;
}

void collision_events_lifecycle(void) {
    sip_collision_events_import();
    const ecs_entity_t a = sip_event_circle(0.0f, 0.0f, true, false);
    const ecs_entity_t b = sip_event_circle(0.75f, 0.0f, true, false);

    ecs_progress();
    test_assert(sip_enter_count == 2);
    test_assert(sip_last_enter.self == a || sip_last_enter.self == b);
    test_assert(sip_last_enter.other == a || sip_last_enter.other == b);
    test_assert(sip_last_enter.self != sip_last_enter.other);
    test_assert(ecs_get_resource_read(SipCollisionStats)->event_pair_count == 1);
    test_assert(ecs_get_resource_read(SipCollisionStats)->event_dispatch_count == 2);

    ecs_progress();
    test_assert(sip_stay_count == 2);
    test_assert(sip_last_stay.penetration >= 0.0f);

    ecs_set(b, Position, { .x = 3.0f, .y = 0.0f });
    ecs_progress();
    test_assert(sip_exit_count == 2);
    test_assert(sip_last_exit.other == b || sip_last_exit.other == a);
    ecs_fini();
}

void collision_events_interest(void) {
    sip_collision_events_import();
    const ecs_entity_t a = sip_event_circle(0.0f, 0.0f, true, false);
    sip_event_circle(0.75f, 0.0f, false, false);
    ecs_progress();
    test_assert(sip_enter_count == 1);
    test_assert(sip_last_enter.self == a);
    test_assert(sip_last_enter.normal_x > 0.0f);
    test_assert(ecs_get_resource_read(SipCollisionStats)->event_pair_count == 1);

    ecs_remove(a, CollisionEvents);
    ecs_progress();
    test_assert(sip_enter_count == 1);
    test_assert(sip_stay_count == 0);
    test_assert(sip_exit_count == 0);
    ecs_fini();

    sip_collision_events_import();
    const ecs_entity_t late = sip_event_circle(0.0f, 0.0f, false, false);
    sip_event_circle(0.75f, 0.0f, false, false);
    ecs_progress();
    test_assert(ecs_get_resource_read(SipCollisionStats)->event_pair_count == 0);
    ecs_add(late, CollisionEvents);
    ecs_progress();
    test_assert(sip_enter_count == 1);
    ecs_fini();
}

void collision_events_sensor(void) {
    sip_collision_events_import();
    const ecs_entity_t circle = sip_event_static_circle(0.0f, 0.0f, true, true);
    const ecs_entity_t box = sip_event_static_box(0.0f, 0.0f, false, false);
    ecs_progress();
    test_assert(sip_enter_count == 1);
    test_assert(ecs_get_resource_read(SipCollisionStats)->contact_count == 1);
    test_assert(ecs_get(circle, Position)->x == 0.0f);
    test_assert(ecs_get(box, Position)->x == 0.0f);
    ecs_fini();

    sip_collision_events_import();
    sip_event_static_circle(0.0f, 0.0f, false, false);
    const ecs_entity_t sensor_box = sip_event_static_box(0.0f, 0.0f, true, true);
    ecs_progress();
    test_assert(sip_enter_count == 1);
    test_assert(sip_last_enter.self == sensor_box);
    ecs_fini();

    sip_collision_events_import();
    sip_event_static_circle(0.0f, 0.0f, true, true);
    sip_event_static_circle(0.75f, 0.0f, true, true);
    ecs_progress();
    test_assert(sip_enter_count == 2);
    test_assert(ecs_get_resource_read(SipCollisionStats)->contact_count == 1);
    ecs_fini();
}

void collision_events_removed_entities(void) {
    sip_collision_events_import();
    const ecs_entity_t a = sip_event_circle(0.0f, 0.0f, true, false);
    const ecs_entity_t b = sip_event_circle(0.75f, 0.0f, true, false);
    ecs_progress();
    test_assert(sip_enter_count == 2);
    ecs_kill(b);
    ecs_progress();
    test_assert(sip_exit_count == 1);
    test_assert(sip_last_exit.self == a);
    test_assert(sip_last_exit.other == b);
    ecs_fini();

    sip_collision_events_import();
    const ecs_entity_t sensor = sip_event_static_circle(0.0f, 0.0f, true, true);
    sip_event_static_box(0.0f, 0.0f, false, false);
    ecs_progress();
    test_assert(sip_enter_count == 1);
    ecs_remove(sensor, CircleCollider);
    ecs_progress();
    test_assert(sip_exit_count == 1);
    ecs_fini();
}

void collision_events_capacity(void) {
    sip_collision_events_import();
    ecs_set_resource(SipCollisionCapacity, {
        .proxy_capacity = 8,
        .pair_capacity = 8,
        .contact_capacity = 8,
        .event_pair_capacity = 8,
    });
    sip_event_circle(0.0f, 0.0f, true, false);
    sip_event_circle(0.75f, 0.0f, true, false);
    sip_event_static_circle(10.0f, 0.0f, false, true);
    ecs_progress();
    const uint64_t growth = ecs_get_resource_read(SipCollisionStats)->scratch_growth_count;
    for (uint32_t i = 0; i < 1000; i++) {
        ecs_progress();
    }
    test_assert(ecs_get_resource_read(SipCollisionStats)->scratch_growth_count == growth);
    ecs_fini();
}
