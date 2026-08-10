#include <test.h>

#include "../../src/collision_internal.h"
#include "../../src/collision_narrowphase.c"

static void assert_close(float actual, float expected) {
    const float delta = actual - expected;
    test_assert((delta < 0.0f ? -delta : delta) < 0.0001f);
}

static SipObb make_box(
    float center_x,
    float center_y,
    float half_width,
    float half_height,
    float angle
) {
    float cosine = 1.0f;
    float sine = 0.0f;
    if (angle == 0.25f) {
        cosine = 0.9689124f;
        sine = 0.24740396f;
    } else if (angle == 0.785398163f) {
        cosine = 0.70710677f;
        sine = 0.70710677f;
    } else if (angle == 0.4f) {
        cosine = 0.92106098f;
        sine = 0.38941833f;
    }
    return (SipObb){
        .center_x = center_x,
        .center_y = center_y,
        .half_width = half_width,
        .half_height = half_height,
        .axis_x_x = cosine,
        .axis_x_y = sine,
        .axis_y_x = -sine,
        .axis_y_y = cosine,
    };
}

void collision_narrowphase_circle_circle(void) {
    SipContact contact;

    test_false(sip_circle_circle(0.0f, 0.0f, 1.0f, 3.0f, 0.0f, 1.0f, &contact));

    test_true(sip_circle_circle(0.0f, 0.0f, 1.0f, 2.0f, 0.0f, 1.0f, &contact));
    assert_close(contact.normal_x, 1.0f);
    assert_close(contact.normal_y, 0.0f);
    assert_close(contact.penetration, 0.0f);

    test_true(sip_circle_circle(0.0f, 0.0f, 1.0f, 1.5f, 0.0f, 1.0f, &contact));
    assert_close(contact.normal_x, 1.0f);
    assert_close(contact.penetration, 0.5f);

    test_true(sip_circle_circle(4.0f, -2.0f, 1.0f, 4.0f, -2.0f, 2.0f, &contact));
    assert_close(contact.normal_x, 1.0f);
    assert_close(contact.normal_y, 0.0f);
    assert_close(contact.penetration, 3.0f);

    test_true(sip_circle_circle(0.0f, 0.0f, 1.0f, 0.75f, 0.5f, 1.0f, &contact));
    const float normal_x = contact.normal_x;
    const float normal_y = contact.normal_y;
    test_true(sip_circle_circle(0.75f, 0.5f, 1.0f, 0.0f, 0.0f, 1.0f, &contact));
    assert_close(contact.normal_x, -normal_x);
    assert_close(contact.normal_y, -normal_y);
}

void collision_narrowphase_circle_box(void) {
    SipContact contact;
    const SipObb box = make_box(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);

    test_true(sip_circle_box(1.5f, 0.0f, 0.75f, &box, &contact));
    assert_close(contact.normal_x, -1.0f);
    assert_close(contact.normal_y, 0.0f);
    assert_close(contact.penetration, 0.25f);

    test_true(sip_circle_box(1.5f, 1.5f, 0.75f, &box, &contact));
    assert_close(contact.normal_x, -0.70710677f);
    assert_close(contact.normal_y, -0.70710677f);

    test_true(sip_circle_box(0.0f, 0.0f, 0.25f, &box, &contact));
    assert_close(contact.normal_x, 1.0f);
    assert_close(contact.normal_y, 0.0f);
    assert_close(contact.penetration, 1.25f);

    const SipObb rotated_box = make_box(0.0f, 0.0f, 1.0f, 0.5f, 0.25f);
    test_true(sip_circle_box(
        rotated_box.axis_x_x * 1.2f,
        rotated_box.axis_x_y * 1.2f,
        0.5f,
        &rotated_box,
        &contact
    ));
    assert_close(contact.normal_x, -rotated_box.axis_x_x);
    assert_close(contact.normal_y, -rotated_box.axis_x_y);

    test_true(sip_circle_box(-1.5f, 0.0f, 0.5f, &box, &contact));
    assert_close(contact.normal_x, 1.0f);
    assert_close(contact.normal_y, 0.0f);

    test_true(sip_circle_box(2.0f, 0.0f, 1.0f, &box, &contact));
    assert_close(contact.penetration, 0.0f);
}

void collision_narrowphase_box_box(void) {
    SipContact contact;
    const SipObb unit = make_box(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);

    const SipObb separated_x = make_box(2.1f, 0.0f, 1.0f, 1.0f, 0.0f);
    test_false(sip_box_box(&unit, &separated_x, &contact));

    const SipObb separated_y = make_box(0.0f, 2.1f, 1.0f, 1.0f, 0.0f);
    test_false(sip_box_box(&unit, &separated_y, &contact));

    const SipObb overlap = make_box(1.5f, 0.0f, 1.0f, 1.0f, 0.0f);
    test_true(sip_box_box(&unit, &overlap, &contact));
    assert_close(contact.normal_x, 1.0f);
    assert_close(contact.normal_y, 0.0f);
    assert_close(contact.penetration, 0.5f);

    const SipObb contained = make_box(0.0f, 0.0f, 0.25f, 0.5f, 0.0f);
    test_true(sip_box_box(&unit, &contained, &contact));
    assert_close(contact.normal_x, 1.0f);
    assert_close(contact.normal_y, 0.0f);

    const SipObb rotated_45 = make_box(0.0f, 0.0f, 1.0f, 1.0f, 0.785398163f);
    test_true(sip_box_box(&unit, &rotated_45, &contact));
    test_assert(contact.penetration > 0.0f);

    const SipObb twice_rotated = make_box(0.75f, 0.25f, 1.0f, 0.5f, 0.4f);
    test_true(sip_box_box(&unit, &twice_rotated, &contact));
    test_assert(contact.normal_x * contact.normal_x + contact.normal_y * contact.normal_y > 0.99f);

    const SipObb tangent = make_box(2.0f, 0.0f, 1.0f, 1.0f, 0.0f);
    test_true(sip_box_box(&unit, &tangent, &contact));
    assert_close(contact.penetration, 0.0f);

    const SipObb same_center = make_box(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);
    test_true(sip_box_box(&unit, &same_center, &contact));
    assert_close(contact.normal_x, 1.0f);
    assert_close(contact.normal_y, 0.0f);
}

void collision_narrowphase_ecs_requirements(void) {
    ecs_init();
    ECS_MODULE_IMPORT(siphysics, { .use_custom_settings = false });

    ecs_entity_t circle = ecs_new();
    ecs_add(circle, CircleCollider);
    ecs_add(circle, Sensor);
    test_true(ecs_has(circle, Sensor));
    test_true(ecs_has(circle, Position));
    test_true(ecs_has(circle, CollisionMaterial));
    test_true(ecs_has(circle, CollisionFilter));
    test_false(ecs_has(circle, Rotation));
    assert_close(ecs_get(circle, CircleCollider)->radius, 0.5f);
    assert_close(ecs_get(circle, CollisionMaterial)->friction, 0.5f);
    assert_close(ecs_get(circle, CollisionMaterial)->restitution, 0.0f);
    test_assert(ecs_get(circle, CollisionFilter)->layer == 1u);
    test_assert(ecs_get(circle, CollisionFilter)->mask == UINT32_MAX);

    ecs_entity_t box = ecs_new();
    ecs_add(box, BoxCollider);
    test_true(ecs_has(box, Position));
    test_true(ecs_has(box, Rotation));
    test_true(ecs_has(box, CollisionMaterial));
    test_true(ecs_has(box, CollisionFilter));
    assert_close(ecs_get(box, BoxCollider)->half_width, 0.5f);
    assert_close(ecs_get(box, BoxCollider)->half_height, 0.5f);

    ecs_fini();
}
