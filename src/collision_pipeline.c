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

static void sip_reserve_event_pairs(SipCollisionRuntime *runtime, uint32_t needed) {
    if (needed <= runtime->event_pair_capacity) {
        return;
    }
    uint32_t capacity = runtime->event_pair_capacity ? runtime->event_pair_capacity * 2 : 64;
    while (capacity < needed) {
        capacity *= 2;
    }
    runtime->previous_pairs = realloc(
        runtime->previous_pairs, sizeof(*runtime->previous_pairs) * capacity);
    runtime->current_pairs = realloc(
        runtime->current_pairs, sizeof(*runtime->current_pairs) * capacity);
    runtime->event_pair_scratch = realloc(
        runtime->event_pair_scratch, sizeof(*runtime->event_pair_scratch) * capacity);
    runtime->event_pair_capacity = capacity;
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
                              uint8_t body_type, bool sensor, bool events) {
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
    batch->entities = it->entities;
    batch->count = it->count;
    batch->shape = shape;
    batch->body_type = body_type;
    batch->sensor = sensor;
    batch->event_enabled = events;

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
        proxy->sensor = batch->sensor;
        proxy->event_interest = batch->event_enabled;
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
                              uint8_t body_type, bool sensor, bool events) {
    ecs_iter_t it = ecs_query_iter(query);
    while (ecs_iter_next(&it)) {
        sip_collect_batch(runtime, &it, layout, shape, body_type, sensor, events);
    }
}

static const SipQueryLayout sip_dynamic_circle_layout = { 0, 1, 2, UINT8_MAX, 3, 4, 5 };
static const SipQueryLayout sip_kinematic_circle_layout = { 0, 1, UINT8_MAX, UINT8_MAX, 2, 3, 4 };
static const SipQueryLayout sip_static_circle_layout = { 0, UINT8_MAX, UINT8_MAX, UINT8_MAX, 1, 2, 3 };
static const SipQueryLayout sip_dynamic_box_layout = { 0, 1, 2, 3, 4, 5, 6 };
static const SipQueryLayout sip_kinematic_box_layout = { 0, 1, UINT8_MAX, 2, 3, 4, 5 };
static const SipQueryLayout sip_static_box_layout = { 0, UINT8_MAX, UINT8_MAX, 1, 2, 3, 4 };

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

static void sip_add_event_pair(SipCollisionRuntime *runtime,
                               const SipProxy *proxy_a,
                               const SipProxy *proxy_b,
                               const SipContact *geometry) {
    if (!(proxy_a->event_interest || proxy_b->event_interest)) {
        return;
    }
    sip_reserve_event_pairs(runtime, runtime->current_event_pair_count + 1);
    const SipBatchRef *batch_a = &runtime->batches[proxy_a->batch_index];
    const SipBatchRef *batch_b = &runtime->batches[proxy_b->batch_index];
    const ecs_entity_t entity_a = batch_a->entities[proxy_a->row];
    const ecs_entity_t entity_b = batch_b->entities[proxy_b->row];
    float point_x = geometry->points[0].x;
    float point_y = geometry->points[0].y;
    float penetration = geometry->points[0].penetration;
    if (geometry->point_count == 2) {
        point_x = (geometry->points[0].x + geometry->points[1].x) * 0.5f;
        point_y = (geometry->points[0].y + geometry->points[1].y) * 0.5f;
        penetration = geometry->points[0].penetration > geometry->points[1].penetration
            ? geometry->points[0].penetration
            : geometry->points[1].penetration;
    }
    SipEventPair *pair = &runtime->current_pairs[runtime->current_event_pair_count++];
    if (entity_a < entity_b) {
        *pair = (SipEventPair){
            .a = entity_a,
            .b = entity_b,
            .normal_x = geometry->normal_x,
            .normal_y = geometry->normal_y,
            .point_x = point_x,
            .point_y = point_y,
            .penetration = penetration,
            .interest_a = proxy_a->event_interest,
            .interest_b = proxy_b->event_interest,
        };
    } else {
        *pair = (SipEventPair){
            .a = entity_b,
            .b = entity_a,
            .normal_x = -geometry->normal_x,
            .normal_y = -geometry->normal_y,
            .point_x = point_x,
            .point_y = point_y,
            .penetration = penetration,
            .interest_a = proxy_b->event_interest,
            .interest_b = proxy_a->event_interest,
        };
    }
}

static void sip_add_contact(SipCollisionRuntime *runtime, const SipProxy *proxy_a,
                            const SipProxy *proxy_b, const SipContactPoint *point,
                            float normal_x, float normal_y) {
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
    contact->normal_x = normal_x;
    contact->normal_y = normal_y;
    contact->point_x = point->x;
    contact->point_y = point->y;
    contact->penetration = point->penetration;
    contact->feature_id = point->feature_id;
    contact->normal_impulse = 0.0f;
    contact->tangent_impulse = 0.0f;
    contact->restitution_bias = 0.0f;
    contact->body_type_a = proxy_a->body_type;
    contact->body_type_b = proxy_b->body_type;
    sip_contact_material(batch_a, proxy_a->row, batch_b, proxy_b->row, contact);

    if (!contact->sensor) {
        const ecs_entity_t entity_a =
            batch_a->entities[proxy_a->row];

        const ecs_entity_t entity_b =
            batch_b->entities[proxy_b->row];

        sip_contact_cache_restore(
            runtime,
            contact,
            entity_a,
            entity_b
        );
    }

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
            sip_add_event_pair(runtime, a, b, &geometry);
            for (uint32_t point_index = 0; point_index < geometry.point_count; point_index++) {
                sip_add_contact(runtime, a, b, &geometry.points[point_index],
                                geometry.normal_x, geometry.normal_y);
            }
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
            sip_add_event_pair(runtime, circle_proxy, box_proxy, &geometry);
            for (uint32_t point_index = 0; point_index < geometry.point_count; point_index++) {
                sip_add_contact(runtime, circle_proxy, box_proxy,
                                &geometry.points[point_index],
                                geometry.normal_x, geometry.normal_y);
            }
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
            sip_add_event_pair(runtime, a, b, &geometry);
            for (uint32_t point_index = 0; point_index < geometry.point_count; point_index++) {
                sip_add_contact(runtime, a, b, &geometry.points[point_index],
                                geometry.normal_x, geometry.normal_y);
            }
        }
    }
}

void sip_collision_collect(SipCollisionRuntime *runtime) {
    sip_collision_runtime_reset(runtime);

    const SipQueryLayout *layouts[] = {
        &sip_dynamic_circle_layout, &sip_kinematic_circle_layout,
        &sip_static_circle_layout, &sip_dynamic_box_layout,
        &sip_kinematic_box_layout, &sip_static_box_layout,
    };
    const uint8_t shapes[] = {
        SIP_SHAPE_CIRCLE, SIP_SHAPE_CIRCLE, SIP_SHAPE_CIRCLE,
        SIP_SHAPE_BOX, SIP_SHAPE_BOX, SIP_SHAPE_BOX,
    };
    const uint8_t body_types[] = {
        SIP_BODY_DYNAMIC, SIP_BODY_KINEMATIC, SIP_BODY_STATIC,
        SIP_BODY_DYNAMIC, SIP_BODY_KINEMATIC, SIP_BODY_STATIC,
    };
    for (uint32_t base = 0; base < 6; base++) {
        for (uint32_t flags = 0; flags < 4; flags++) {
            const bool sensor = (flags & 2u) != 0;
            const bool events = (flags & 1u) != 0;
            sip_collect_query(runtime, runtime->queries[base * 4 + flags], layouts[base],
                              shapes[base], body_types[base], sensor, events);
        }
    }
}

void sip_collision_narrowphase(SipCollisionRuntime *runtime) {
    narrow_cc(runtime);
    narrow_cb(runtime);
    narrow_bb(runtime);
}

void siphysics_collision_step(void) {
    SipCollisionRuntime *runtime = ecs_get_resource(SipCollisionRuntime);
    SipCollisionStats *stats = ecs_get_resource(SipCollisionStats);
    const SipSettings *settings = ecs_get_resource_read(SipSettings);
    const SipCollisionCapacity *capacity = ecs_get_resource_read(SipCollisionCapacity);
    sip_collision_runtime_reserve(runtime, capacity);
    sip_collision_collect(runtime);
    sip_broadphase(runtime);
    sip_collision_narrowphase(runtime);
    sip_collision_solve(runtime, settings);
    sip_collision_event_diff(runtime);

    stats->proxy_count = runtime->proxy_count;
    stats->candidate_count = runtime->circle_circle_count + runtime->circle_box_count +
                             runtime->box_box_count;
    stats->contact_count = runtime->contact_count;
    stats->contact_cache_count = runtime->current_contact_cache_count;
    stats->contact_cache_hit_count = runtime->contact_cache_hit_count;
    stats->event_pair_count = runtime->current_event_pair_count;
    stats->event_dispatch_count = runtime->event_dispatch_count;
    stats->scratch_growth_count = runtime->growth_count;
}
