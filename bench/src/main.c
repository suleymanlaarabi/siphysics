#define _POSIX_C_SOURCE 200809L

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "../../src/collision_internal.h"
#include "../../src/collision_narrowphase.c"

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

    free(box_pairs);
    free(circle_box_pairs);
    free(circle_pairs);
    return 0;
}
