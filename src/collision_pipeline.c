#include "collision_runtime.h"
#include "siecs.h"
#include <stdlib.h>

typedef struct SipQueryLayout {
    uint8_t position;
    uint8_t velocity;
    uint8_t inverse_mass;
    uint8_t rotation;
    uint8_t shape;
    uint8_t material;
    uint8_t filter;
    uint8_t sensor;
} SipQueryLayout;

static void sip_reserve_batch(SipCollisionRuntime *runtime, uint32_t needed) {
    if (needed <= runtime->batch_capacity) {
        return;
    }
    uint32_t capacity = runtime->batch_capacity ? runtime->batch_capacity : 16;
    while (capacity < needed) {
        capacity *= 2;
    }
    runtime->batches = realloc(runtime->batches, sizeof(*runtime->batches) * capacity);
    runtime->batch_capacity = capacity;
    runtime->growth_count++;
}

static void sip_reserve_proxy(SipCollisionRuntime *runtime, uint32_t needed) {
    if (needed <= runtime->proxy_capacity) {
        return;
    }
    uint32_t capacity = runtime->proxy_capacity ? runtime->proxy_capacity : 64;
    while (capacity < needed) {
        capacity *= 2;
    }
    runtime->proxies = realloc(runtime->proxies, sizeof(*runtime->proxies) * capacity);
    runtime->proxy_scratch = realloc(runtime->proxy_scratch, sizeof(*runtime->proxy_scratch) * capacity);
    runtime->box_geoms = realloc(runtime->box_geoms, sizeof(*runtime->box_geoms) * capacity);
    runtime->proxy_capacity = capacity;
    runtime->growth_count++;
}

static SipObb sip_box_from_batch(const SipBatchRef *batch, uint32_t row,
                                 const SipBoxGeom *geom) {
    const Position *position = &batch->positions[row];
    const BoxCollider *box = &batch->boxes[row];
    return (SipObb){
        .center_x = position->x,
        .center_y = position->y,
        .half_width = box->half_width,
        .half_height = box->half_height,
        .axis_x_x = geom->axis_x_x,
        .axis_x_y = geom->axis_x_y,
        .axis_y_x = geom->axis_y_x,
        .axis_y_y = geom->axis_y_y,
    };
}

static void sip_collect_batch(SipCollisionRuntime *runtime, ecs_iter_t *it,
                              const SipQueryLayout *layout, uint8_t shape,
                              uint8_t body_type) {
    SipBatchRef *batch;
    sip_reserve_batch(runtime, runtime->batch_count + 1);
    batch = &runtime->batches[runtime->batch_count++];
    batch->positions = ecs_field(it, layout->position);
    batch->velocities = layout->velocity == UINT8_MAX ? NULL : ecs_field(it, layout->velocity);
    batch->inverse_masses = layout->inverse_mass == UINT8_MAX ? NULL : ecs_field(it, layout->inverse_mass);
    batch->rotations = layout->rotation == UINT8_MAX ? NULL : ecs_field(it, layout->rotation);
    batch->circles = shape == SIP_SHAPE_CIRCLE ? ecs_field(it, layout->shape) : NULL;
    batch->boxes = shape == SIP_SHAPE_BOX ? ecs_field(it, layout->shape) : NULL;
    batch->materials = ecs_field(it, layout->material);
    batch->filters = ecs_field(it, layout->filter);
    batch->count = it->count;
    batch->shape = shape;
    batch->body_type = body_type;
    batch->sensor = ecs_field_kind(it, layout->sensor) != EcsFieldNone;

    for (uint32_t row = 0; row < batch->count; row++) {
        sip_reserve_proxy(runtime, runtime->proxy_count + 1);
        const uint32_t proxy_index = runtime->proxy_count++;
        SipProxy *proxy = &runtime->proxies[proxy_index];
        const Position *position = &batch->positions[row];
        const CollisionFilter *filter = &batch->filters[row];
        proxy->batch_index = runtime->batch_count - 1;
        proxy->row = row;
        proxy->layer = filter->layer;
        proxy->mask = filter->mask;
        proxy->shape = shape;
        proxy->body_type = body_type;
        proxy->box_geom_index = UINT32_MAX;

        if (shape == SIP_SHAPE_CIRCLE) {
            const float radius = batch->circles[row].radius;
            proxy->min_x = position->x - radius;
            proxy->max_x = position->x + radius;
            proxy->min_y = position->y - radius;
            proxy->max_y = position->y + radius;
        } else {
            float sine;
            float cosine;
            sip_sincosf(batch->rotations[row].angle, &sine, &cosine);
            const SipBoxGeom geom = {
                .axis_x_x = cosine,
                .axis_x_y = sine,
                .axis_y_x = -sine,
                .axis_y_y = cosine,
            };
            runtime->box_geoms[proxy_index] = geom;
            proxy->box_geom_index = proxy_index;
            const BoxCollider *box = &batch->boxes[row];
            const float extent_x = sip_absf(geom.axis_x_x) * box->half_width +
                                   sip_absf(geom.axis_y_x) * box->half_height;
            const float extent_y = sip_absf(geom.axis_x_y) * box->half_width +
                                   sip_absf(geom.axis_y_y) * box->half_height;
            proxy->min_x = position->x - extent_x;
            proxy->max_x = position->x + extent_x;
            proxy->min_y = position->y - extent_y;
            proxy->max_y = position->y + extent_y;
        }
    }
}

static void sip_collect_query(SipCollisionRuntime *runtime, ecs_query_id_t query,
                              const SipQueryLayout *layout, uint8_t shape,
                              uint8_t body_type) {
    ecs_iter_t it = ecs_query_iter(query);
    while (ecs_iter_next(&it)) {
        sip_collect_batch(runtime, &it, layout, shape, body_type);
    }
}

static const SipQueryLayout sip_dynamic_circle_layout = { 0, 1, 2, UINT8_MAX, 3, 4, 5, 6 };
static const SipQueryLayout sip_kinematic_circle_layout = { 0, 1, UINT8_MAX, UINT8_MAX, 2, 3, 4, 5 };
static const SipQueryLayout sip_static_circle_layout = { 0, UINT8_MAX, UINT8_MAX, UINT8_MAX, 1, 2, 3, 4 };
static const SipQueryLayout sip_dynamic_box_layout = { 0, 1, 2, 3, 4, 5, 6, 7 };
static const SipQueryLayout sip_kinematic_box_layout = { 0, 1, UINT8_MAX, 2, 3, 4, 5, 6 };
static const SipQueryLayout sip_static_box_layout = { 0, UINT8_MAX, UINT8_MAX, 1, 2, 3, 4, 5 };

static void sip_contact_material(const SipBatchRef *a, uint32_t row_a,
                                 const SipBatchRef *b, uint32_t row_b,
                                 SipSolverContact *contact) {
    const CollisionMaterial *material_a = &a->materials[row_a];
    const CollisionMaterial *material_b = &b->materials[row_b];
    const float friction_a = sip_clampf(material_a->friction, 0.0f, 1.0f);
    const float friction_b = sip_clampf(material_b->friction, 0.0f, 1.0f);
    const float restitution_a = sip_clampf(material_a->restitution, 0.0f, 1.0f);
    const float restitution_b = sip_clampf(material_b->restitution, 0.0f, 1.0f);
    contact->friction = sip_sqrtf(friction_a * friction_b);
    contact->restitution = restitution_a > restitution_b ? restitution_a : restitution_b;
    contact->sensor = a->sensor || b->sensor;
}

static void sip_add_contact(SipCollisionRuntime *runtime, const SipProxy *proxy_a,
                            const SipProxy *proxy_b, const SipContact *geometry) {
    if (runtime->contact_count == runtime->contact_capacity) {
        uint32_t capacity = runtime->contact_capacity ? runtime->contact_capacity * 2 : 64;
        runtime->contacts = realloc(runtime->contacts, sizeof(*runtime->contacts) * capacity);
        runtime->contact_capacity = capacity;
        runtime->growth_count++;
    }
    SipSolverContact *contact = &runtime->contacts[runtime->contact_count++];
    const SipBatchRef *batch_a = &runtime->batches[proxy_a->batch_index];
    const SipBatchRef *batch_b = &runtime->batches[proxy_b->batch_index];
    contact->batch_a = proxy_a->batch_index;
    contact->row_a = proxy_a->row;
    contact->batch_b = proxy_b->batch_index;
    contact->row_b = proxy_b->row;
    contact->normal_x = geometry->normal_x;
    contact->normal_y = geometry->normal_y;
    contact->penetration = geometry->penetration;
    contact->normal_impulse = 0.0f;
    contact->tangent_impulse = 0.0f;
    contact->body_type_a = proxy_a->body_type;
    contact->body_type_b = proxy_b->body_type;
    sip_contact_material(batch_a, proxy_a->row, batch_b, proxy_b->row, contact);
}

static void narrow_cc(SipCollisionRuntime *runtime) {
    for (uint32_t i = 0; i < runtime->circle_circle_count; i++) {
        const SipPair *pair = &runtime->circle_circle_pairs[i];
        const SipProxy *a = &runtime->proxies[pair->proxy_a];
        const SipProxy *b = &runtime->proxies[pair->proxy_b];
        const SipBatchRef *batch_a = &runtime->batches[a->batch_index];
        const SipBatchRef *batch_b = &runtime->batches[b->batch_index];
        SipContact geometry;
        if (sip_circle_circle(
                batch_a->positions[a->row].x, batch_a->positions[a->row].y,
                batch_a->circles[a->row].radius,
                batch_b->positions[b->row].x, batch_b->positions[b->row].y,
                batch_b->circles[b->row].radius, &geometry)) {
            sip_add_contact(runtime, a, b, &geometry);
        }
    }
}

static void narrow_cb(SipCollisionRuntime *runtime) {
    for (uint32_t i = 0; i < runtime->circle_box_count; i++) {
        const SipPair *pair = &runtime->circle_box_pairs[i];
        const SipProxy *circle_proxy = &runtime->proxies[pair->proxy_a];
        const SipProxy *box_proxy = &runtime->proxies[pair->proxy_b];
        const SipBatchRef *circle_batch = &runtime->batches[circle_proxy->batch_index];
        const SipBatchRef *box_batch = &runtime->batches[box_proxy->batch_index];
        const SipObb box = sip_box_from_batch(
            box_batch, box_proxy->row, &runtime->box_geoms[box_proxy->box_geom_index]);
        SipContact geometry;
        if (sip_circle_box(
                circle_batch->positions[circle_proxy->row].x,
                circle_batch->positions[circle_proxy->row].y,
                circle_batch->circles[circle_proxy->row].radius, &box, &geometry)) {
            sip_add_contact(runtime, circle_proxy, box_proxy, &geometry);
        }
    }
}

static void narrow_bb(SipCollisionRuntime *runtime) {
    for (uint32_t i = 0; i < runtime->box_box_count; i++) {
        const SipPair *pair = &runtime->box_box_pairs[i];
        const SipProxy *a = &runtime->proxies[pair->proxy_a];
        const SipProxy *b = &runtime->proxies[pair->proxy_b];
        const SipBatchRef *batch_a = &runtime->batches[a->batch_index];
        const SipBatchRef *batch_b = &runtime->batches[b->batch_index];
        const SipObb box_a = sip_box_from_batch(
            batch_a, a->row, &runtime->box_geoms[a->box_geom_index]);
        const SipObb box_b = sip_box_from_batch(
            batch_b, b->row, &runtime->box_geoms[b->box_geom_index]);
        SipContact geometry;
        if (sip_box_box(&box_a, &box_b, &geometry)) {
            sip_add_contact(runtime, a, b, &geometry);
        }
    }
}

void sip_collision_collect(SipCollisionRuntime *runtime) {
    sip_collision_runtime_reset(runtime);

    sip_collect_query(runtime, runtime->queries[0], &sip_dynamic_circle_layout,
                      SIP_SHAPE_CIRCLE, SIP_BODY_DYNAMIC);
    sip_collect_query(runtime, runtime->queries[1], &sip_kinematic_circle_layout,
                      SIP_SHAPE_CIRCLE, SIP_BODY_KINEMATIC);
    sip_collect_query(runtime, runtime->queries[2], &sip_static_circle_layout,
                      SIP_SHAPE_CIRCLE, SIP_BODY_STATIC);
    sip_collect_query(runtime, runtime->queries[3], &sip_dynamic_box_layout,
                      SIP_SHAPE_BOX, SIP_BODY_DYNAMIC);
    sip_collect_query(runtime, runtime->queries[4], &sip_kinematic_box_layout,
                      SIP_SHAPE_BOX, SIP_BODY_KINEMATIC);
    sip_collect_query(runtime, runtime->queries[5], &sip_static_box_layout,
                      SIP_SHAPE_BOX, SIP_BODY_STATIC);
}

void sip_collision_narrowphase(SipCollisionRuntime *runtime) {
    narrow_cc(runtime);
    narrow_cb(runtime);
    narrow_bb(runtime);
}

void siphysics_collision_step(ecs_iter_t *it) {
    (void)it;
    SipCollisionRuntime *runtime = ecs_get_resource(SipCollisionRuntime);
    SipCollisionStats *stats = ecs_get_resource(SipCollisionStats);
    const SipSettings *settings = ecs_get_resource_read(SipSettings);
    sip_collision_collect(runtime);
    sip_broadphase(runtime);
    sip_collision_narrowphase(runtime);
    sip_collision_solve(runtime, settings);

    stats->proxy_count = runtime->proxy_count;
    stats->candidate_count = runtime->circle_circle_count + runtime->circle_box_count +
                             runtime->box_box_count;
    stats->contact_count = runtime->contact_count;
    stats->scratch_growth_count = runtime->growth_count;
}
