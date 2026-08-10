#define _POSIX_C_SOURCE 200809L

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define SIPHYSICS_BENCHMARK
#include "../../src/collision_internal.h"
#include "../../src/collision_runtime.h"
#include "../../src/collision_narrowphase.c"
#include "../../src/collision_runtime.c"
#include "../../src/collision_broadphase.c"
#include "../../src/collision_solver.c"
#include "../../src/collision_pipeline.c"
#include <siphysics/physics.h>

enum { sip_bench_count = 1000000 };

typedef struct CirclePair {
    float ax;
    float ay;
    float ar;
    float bx;
    float by;
    float br;
} CirclePair;

typedef struct CircleBoxPair {
    float cx;
    float cy;
    float radius;
    SipObb box;
} CircleBoxPair;

typedef struct BoxPair {
    SipObb a;
    SipObb b;
} BoxPair;

typedef struct BenchBodyHot {
    Position position;
    Velocity velocity;
} BenchBodyHot;

static uint64_t now_ns(void) {
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (uint64_t)time.tv_sec * 1000000000ull + (uint64_t)time.tv_nsec;
}

static SipObb bench_box(float center_x, float center_y, float angle) {
    const float cosine_values[] = {
        1.0f, 0.9950042f, 0.9800666f, 0.9553365f,
        0.9210610f, 0.8775826f, 0.8253356f, 0.7648422f,
        0.6967067f, 0.6216099f, 0.5403023f, 0.4535961f,
        0.3623578f, 0.2674988f, 0.1699671f, 0.0707372f,
    };
    const float sine_values[] = {
        0.0f, 0.0998334f, 0.1986693f, 0.2955202f,
        0.3894183f, 0.4794255f, 0.5646425f, 0.6442177f,
        0.7173561f, 0.7833269f, 0.8414710f, 0.8912074f,
        0.9320391f, 0.9635582f, 0.9854497f, 0.9974950f,
    };
    const uint32_t rotation_index = (uint32_t)(angle * 10.0f) & 15u;
    const float cosine = cosine_values[rotation_index];
    const float sine = sine_values[rotation_index];
    return (SipObb){
        .center_x = center_x,
        .center_y = center_y,
        .half_width = 1.0f,
        .half_height = 1.0f,
        .axis_x_x = cosine,
        .axis_x_y = sine,
        .axis_y_x = -sine,
        .axis_y_y = cosine,
    };
}

static void prepare_circle_pairs(CirclePair *pairs, int mode) {
    for (uint32_t i = 0; i < sip_bench_count; i++) {
        const bool colliding = mode == 0 ? (i % 5u == 0u)
                              : mode == 1 ? (i % 5u != 0u)
                                          : (i % 2u == 0u);
        pairs[i] = (CirclePair){
            .ax = 0.0f,
            .ay = 0.0f,
            .ar = 0.5f,
            .bx = colliding ? 0.75f : 2.0f,
            .by = 0.0f,
            .br = 0.5f,
        };
    }
}

static void prepare_circle_box_pairs(CircleBoxPair *pairs, int mode) {
    for (uint32_t i = 0; i < sip_bench_count; i++) {
        const bool colliding = mode == 0 ? (i % 5u == 0u)
                              : mode == 1 ? (i % 5u != 0u)
                                          : (i % 2u == 0u);
        const float angle = (float)(i % 16u) * 0.1f;
        pairs[i] = (CircleBoxPair){
            .cx = colliding ? 1.25f : 2.0f,
            .cy = 0.0f,
            .radius = 0.5f,
            .box = bench_box(0.0f, 0.0f, angle),
        };
    }
}

static void prepare_box_pairs(BoxPair *pairs, int mode) {
    for (uint32_t i = 0; i < sip_bench_count; i++) {
        const bool colliding = mode == 0 ? (i % 5u == 0u)
                              : mode == 1 ? (i % 5u != 0u)
                                          : (i % 2u == 0u);
        const float angle_a = (float)(i % 16u) * 0.1f;
        const float angle_b = (float)(i % 11u) * -0.07f;
        pairs[i] = (BoxPair){
            .a = bench_box(0.0f, 0.0f, angle_a),
            .b = bench_box(colliding ? 1.0f : 3.0f, 0.25f, angle_b),
        };
    }
}

static void print_result(const char *name, uint64_t elapsed, uint64_t hits, double checksum) {
    const double tests_per_second = (double)sip_bench_count * 1000000000.0 / (double)elapsed;
    const double nanoseconds_per_test = (double)elapsed / (double)sip_bench_count;
    printf(
        "%s: %.2f ns/test, %.2f tests/s, hits=%" PRIu64 ", checksum=%.5f\n",
        name,
        nanoseconds_per_test,
        tests_per_second,
        hits,
        checksum
    );
}

/* Benchmark-only AoS comparison: includes input and output copies, never used by production. */
static void benchmark_aos_copy(uint32_t capacity) {
    BenchBodyHot *body_hot = malloc(sizeof(*body_hot) * capacity);
    ecs_query_id_t query = ecs_query({
        .terms = {
            ecs_inout(Position),
            ecs_inout(Velocity),
            ecs_in(CircleCollider),
            ecs_filter(Dynamic),
            ecs_not(Kinematic),
            ecs_not(Static),
            ecs_not(BoxCollider),
        }
    });
    uint32_t count = 0;
    const uint64_t start = now_ns();
    ecs_iter_t it = ecs_query_iter(query);
    while (ecs_iter_next(&it)) {
        Position *positions = ecs_field(&it, 0);
        Velocity *velocities = ecs_field(&it, 1);
        for (uint32_t row = 0; row < it.count; row++) {
            body_hot[count++] = (BenchBodyHot){ positions[row], velocities[row] };
        }
    }
    it = ecs_query_iter(query);
    uint32_t row_count = 0;
    while (ecs_iter_next(&it)) {
        Position *positions = ecs_field(&it, 0);
        Velocity *velocities = ecs_field(&it, 1);
        for (uint32_t row = 0; row < it.count; row++) {
            const BenchBodyHot body = body_hot[row_count++];
            positions[row] = body.position;
            velocities[row] = body.velocity;
        }
    }
    const uint64_t elapsed = now_ns() - start;
    printf("AoS copy comparison: %.2f ns/body (input+output), bodies=%u\n",
           count ? (double)elapsed / (double)count : 0.0, count);
    ecs_query_fini(query);
    free(body_hot);
}

static void benchmark_circle_circle(const CirclePair *pairs) {
    uint64_t hits = 0;
    double checksum = 0.0;
    SipContact contact;
    const uint64_t start = now_ns();
    for (uint32_t i = 0; i < sip_bench_count; i++) {
        if (sip_circle_circle(
                pairs[i].ax, pairs[i].ay, pairs[i].ar,
                pairs[i].bx, pairs[i].by, pairs[i].br, &contact)) {
            hits++;
            checksum += contact.penetration;
        }
    }
    print_result("Circle/Circle", now_ns() - start, hits, checksum);
}

static void benchmark_circle_box(const CircleBoxPair *pairs) {
    uint64_t hits = 0;
    double checksum = 0.0;
    SipContact contact;
    const uint64_t start = now_ns();
    for (uint32_t i = 0; i < sip_bench_count; i++) {
        if (sip_circle_box(pairs[i].cx, pairs[i].cy, pairs[i].radius, &pairs[i].box, &contact)) {
            hits++;
            checksum += contact.penetration;
        }
    }
    print_result("Circle/Box", now_ns() - start, hits, checksum);
}

static void benchmark_box_box(const BoxPair *pairs) {
    uint64_t hits = 0;
    double checksum = 0.0;
    SipContact contact;
    const uint64_t start = now_ns();
    for (uint32_t i = 0; i < sip_bench_count; i++) {
        if (sip_box_box(&pairs[i].a, &pairs[i].b, &contact)) {
            hits++;
            checksum += contact.penetration;
        }
    }
    print_result("Box/Box", now_ns() - start, hits, checksum);
}

static void benchmark_pipeline_scale(uint32_t count) {
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
                .penetration_slop = 0.005f,
                .penetration_correction = 0.8f,
            },
        }
    );

    for (uint32_t i = 0; i < count; i++) {
        const float x = (float)i * 4.0f;
        const bool box = (i & 3u) == 0u;
        ecs_entity_t dynamic = ecs_new();
        ecs_add(dynamic, Dynamic);
        if (box) {
            ecs_add(dynamic, BoxCollider);
            ecs_set(dynamic, Rotation, { .angle = (float)(i & 15u) * 0.1f });
        } else {
            ecs_add(dynamic, CircleCollider);
        }
        ecs_set(dynamic, Position, { .x = x, .y = 0.0f });

        ecs_entity_t static_body = ecs_new();
        ecs_add(static_body, Static);
        if (box) {
            ecs_add(static_body, BoxCollider);
            ecs_set(static_body, Rotation, { .angle = (float)(i & 7u) * -0.08f });
        } else {
            ecs_add(static_body, CircleCollider);
        }
        ecs_set(static_body, Position, { .x = x + 0.75f, .y = 0.0f });
    }

    SipCollisionRuntime *runtime = ecs_get_resource(SipCollisionRuntime);
    const SipSettings *settings = ecs_get_resource_read(SipSettings);
    for (uint32_t warmup = 0; warmup < 2; warmup++) {
        sip_collision_collect(runtime);
        sip_broadphase(runtime);
        sip_collision_narrowphase(runtime);
    }
    const uint64_t warmup_growth = runtime->growth_count;

    sip_collision_runtime_reset(runtime);
    const uint64_t total_start = now_ns();
    const uint64_t proxy_start = now_ns();
    sip_collision_collect(runtime);
    const uint64_t proxy_elapsed = now_ns() - proxy_start;
    const uint64_t sort_start = now_ns();
    sip_radix_sort(runtime);
    const uint64_t sort_elapsed = now_ns() - sort_start;
    const uint64_t sap_start = now_ns();
    sip_generate_pairs(runtime);
    const uint64_t sap_elapsed = now_ns() - sap_start;
    const uint64_t narrow_start = now_ns();
    sip_collision_narrowphase(runtime);
    const uint64_t narrow_elapsed = now_ns() - narrow_start;
    const uint64_t solver_start = now_ns();
    sip_collision_solve(runtime, settings);
    const uint64_t solver_elapsed = now_ns() - solver_start;
    const uint64_t total_elapsed = now_ns() - total_start;
    const uint32_t candidates = runtime->circle_circle_count + runtime->circle_box_count +
                                runtime->box_box_count;
    printf(
        "Pipeline %u: proxy %.2f ns/body, sort %.2f ns/body, SAP %.2f ns/body, "
        "narrow %.2f ns/candidate, solver %.2f ns/contact, total %.2f ns/body, "
        "proxies=%u candidates=%u contacts=%u growth=%" PRIu64 "->%" PRIu64 " (delta=%" PRIu64 ")\n",
        count,
        (double)proxy_elapsed / (double)count,
        (double)sort_elapsed / (double)count,
        (double)sap_elapsed / (double)count,
        candidates ? (double)narrow_elapsed / (double)candidates : 0.0,
        runtime->contact_count ? (double)solver_elapsed / (double)runtime->contact_count : 0.0,
        (double)total_elapsed / (double)count,
        runtime->proxy_count,
        candidates,
        runtime->contact_count,
        warmup_growth,
        runtime->growth_count,
        runtime->growth_count - warmup_growth
    );
    if (count == 100000) {
        benchmark_aos_copy(count);
    }
    ecs_fini();
}

int main(void) {
    CirclePair *circle_pairs = malloc(sizeof(*circle_pairs) * sip_bench_count);
    CircleBoxPair *circle_box_pairs = malloc(sizeof(*circle_box_pairs) * sip_bench_count);
    BoxPair *box_pairs = malloc(sizeof(*box_pairs) * sip_bench_count);
    const char *modes[] = { "mostly rejected", "mostly colliding", "mixed" };

    for (int mode = 0; mode < 3; mode++) {
        printf("[%s]\n", modes[mode]);
        prepare_circle_pairs(circle_pairs, mode);
        prepare_circle_box_pairs(circle_box_pairs, mode);
        prepare_box_pairs(box_pairs, mode);
        benchmark_circle_circle(circle_pairs);
        benchmark_circle_box(circle_box_pairs);
        benchmark_box_box(box_pairs);
    }

    printf("[ECS pipeline scales]\n");
    benchmark_pipeline_scale(1000);
    benchmark_pipeline_scale(10000);
    benchmark_pipeline_scale(50000);
    benchmark_pipeline_scale(100000);

    free(box_pairs);
    free(circle_box_pairs);
    free(circle_pairs);
    return 0;
}
