/*
 * Copyright © 2013,2014 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#define _GNU_SOURCE
#include <uv.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <linux/perf_event.h>
#include <asm/unistd.h>
#include <sys/mman.h>
#include <inttypes.h>
#include <sys/mount.h>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <sys/timerfd.h>

#include "intel_bufmgr.h"
#include "intel_batchbuffer.h"
#include "ioctl_wrappers.h"
#include "intel_chipset.h"

#if defined(__i386__)
#define rmb()           asm volatile("lock; addl $0,0(%%esp)" ::: "memory")
#define wmb()           asm volatile("lock; addl $0,0(%%esp)" ::: "memory")
#endif

#if defined(__x86_64__)
#define rmb()           asm volatile("lfence" ::: "memory")
#define wmb()           asm volatile("sfence" ::: "memory")
#endif

/* attr.config */

#define I915_PERF_OA_CTX_ID_MASK	    0xffffffff
#define I915_PERF_OA_SINGLE_CONTEXT_ENABLE  (1ULL << 32)

#define I915_PERF_OA_FORMAT_SHIFT	    33
#define I915_PERF_OA_FORMAT_MASK	    (0x7ULL << 33)
#define I915_PERF_OA_FORMAT_A13_HSW	    (0ULL << 33)
#define I915_PERF_OA_FORMAT_A29_HSW	    (1ULL << 33)
#define I915_PERF_OA_FORMAT_A13_B8_C8_HSW   (2ULL << 33)
#define I915_PERF_OA_FORMAT_A29_B8_C8_HSW   (3ULL << 33)
#define I915_PERF_OA_FORMAT_B4_C8_HSW	    (4ULL << 33)
#define I915_PERF_OA_FORMAT_A45_B8_C8_HSW   (5ULL << 33)
#define I915_PERF_OA_FORMAT_B4_C8_A16_HSW   (6ULL << 33)
#define I915_PERF_OA_FORMAT_C4_B8_HSW	    (7ULL << 33)

#define I915_PERF_OA_TIMER_EXPONENT_SHIFT   36
#define I915_PERF_OA_TIMER_EXPONENT_MASK    (0x3fULL << 36)

#define MAX_EVENTS 256

/* The largest OA circular buffer size for Haswell is 16MB and the
 * smallest report format is 64 bytes.
 */
#define MAX_OABUFFER_SAMPLES ((16 * 1024 *1024) / 64)

/* The smallest sampling period is 160 nanoseconds on Haswell */
#define HIGHEST_SAMPLE_FREQ      (1000000000 / 160)

#define SCRATCH_BO_WIDTH 512
#define SCRATCH_BO_STRIDE (SCRATCH_BO_WIDTH*4)
#define SCRATCH_BO_HEIGHT 512
#define SCRATCH_BO_SIZE (SCRATCH_BO_HEIGHT * SCRATCH_BO_STRIDE)

/* FIXME: HACK to dig out the context id from the
 * otherwise opaque drm_intel_context struct! */
struct _drm_intel_context {
        unsigned int ctx_id;
};

struct intel_device {
        uint32_t device;
        uint32_t subsystem_device;
        uint32_t subsystem_vendor;
};

enum {
        FREQ_EVENT,
        N_EVENT,
};

#if 0
static int hsw_perf_format_sizes[] = {
	[I915_PERF_OA_FORMAT_A13_HSW] = 64,
	[I915_PERF_OA_FORMAT_A29_HSW] = 128,
	[I915_PERF_OA_FORMAT_A13_B8_C8_HSW] = 128,
	[I915_PERF_OA_FORMAT_A29_B8_C8_HSW] = 192,
	[I915_PERF_OA_FORMAT_B4_C8_HSW] = 64,
	[I915_PERF_OA_FORMAT_A45_B8_C8_HSW] = 256,
	[I915_PERF_OA_FORMAT_B4_C8_A16_HSW] = 128,
	[I915_PERF_OA_FORMAT_C4_B8_HSW] = 64
};
#endif

#define MIN_REPORT_SIZE 65
#define MAX_REPORT_SIZE 256
#define MAX_REPORT_LEN 64

/* Samples read from the perf circular buffer */
struct oa_perf_sample {
        struct perf_event_header header;
        uint64_t time; /* PERF_SAMPLE_TIME */
        uint64_t value; /* PERF_SAMPLE_READ */
        uint32_t raw_size;
        uint8_t raw_data[];
};

struct oa_sample {
        uint64_t log_timestamp; /* time OA report was forwarded to userspace */
        uint64_t nominal_value;
        uint32_t report[MAX_REPORT_LEN];
};

struct sample_array
{
        struct oa_sample *samples;
        int space;
        int len;
};

struct event {

        int cpu;

        int fd;

        union {
                struct perf_event_mmap_page *mmap_page;
                uint8_t *mmap_base;
        };
        unsigned int mmap_size;

        struct sample_array array;
};

struct value {
        /* some counters are calculated as a derivative so we
         * want to return an new timestamp... */
        uint64_t timestamp;
        uint64_t value;
};

static struct intel_device intel_dev;

//static unsigned int n_cpus;
static unsigned int page_size;

static uint8_t *scratch_buffer;
static unsigned scratch_buffer_size;

static int epoll_fd = -1;
static int sigint_fd;

static struct event *i915_oa_event;

static struct event *events[MAX_EVENTS + 1];
static int n_events;

#define TAKEN(HEAD, TAIL, POT_SIZE)	(((HEAD) - (TAIL)) & (POT_SIZE - 1))

/* Note: this will equate to 0 when the buffer is exactly full... */
#define REMAINING(HEAD, TAIL, POT_SIZE) (POT_SIZE - TAKEN (HEAD, TAIL, POT_SIZE))

static void sample_array_init(struct sample_array *array)
{
        /* We want to do analysis over ~ 16ms intervals to try and
         * be representative of a whole frame (assuming ~60fps)
         *
         * Being conservative we start with enough space to track
         * samples captured at the highest possible frequency for
         * 100 milliseconds.
         */
        array->space = (HIGHEST_SAMPLE_FREQ / 10);
        array->samples = malloc(array->space * sizeof (struct oa_sample));
        array->len = 0;
}

static struct oa_sample *sample_array_append(struct sample_array *array)
{
        if (array->len == array->space) {
                size_t new_size;

                array->space *= 1.7;
                new_size = sizeof(struct oa_sample) * array->space;
                array->samples = realloc(array->samples, new_size);
        }

        return array->samples + array->len++;
}

static void sample_array_destroy(struct sample_array *array)
{
        free(array->samples);
}

/* In dependency order, such that later counters
 * can be derived and normalized using earlier
 * counters. */
enum common_counter_id {
        GPU_TIMESTAMP,

        GPU_CORE_CLOCK,

        AGGREGATE_CORE_ARRAYS_ACTIVE,
        AGGREGATE_CORE_ARRAYS_STALLED,

        VS_ACTIVE_TIME,
        VS_STALL_TIME,
        NUM_VS_THREADS_LOADED,

        HS_ACTIVE_TIME,
        HS_STALL_TIME,
        NUM_HS_THREADS_LOADED,

        DS_ACTIVE_TIME,
        DS_STALL_TIME,
        NUM_DS_THREADS_LOADED,

        CS_ACTIVE_TIME,
        CS_STALL_TIME,
        NUM_CS_THREADS_LOADED,

        GS_ACTIVE_TIME,
        GS_STALL_TIME,
        NUM_GS_THREADS_LOADED,

        PS_ACTIVE_TIME,
        PS_STALL_TIME,
        NUM_PS_THREADS_LOADED,

        HIZ_FAST_Z_PASSING,
        HIZ_FAST_Z_FAILING,

        SLOW_Z_FAILING,

        PIXEL_KILL_COUNT,

        ALPHA_TEST_FAILED,
        POST_PS_STENCIL_TEST_FAILED,
        POST_PS_Z_TEST_FAILED,

        RENDER_TARGET_WRITES,

        RENDER_ENGINE_BUSY,

        VS_BOTTLENECK,
        GS_BOTTLENECK,

        N_COMMON_COUNTERS,

        EXTRA_COUNTER
};

const char *common_counter_names[] = {
        "GPU Timestamp",
        "GPU Core Clocks",
        "Aggregated Core Array Active",
        "Aggregated Core Array Stalled",
        "Vertex Shader Active Time",
        "Vertex Shader Stall Time",
        "# VS threads loaded",
        "Hull Shader Active Time",
        "Hull Shader Stall Time",
        "# HS threads loaded",
        "Domain Shader Active Time",
        "Domain Shader Stall Time",
        "# DS threads loaded",
        "Compute Shader Active Time",
        "Compute Shader Stall Time",
        "# CS threads loaded",
        "Geometry Shader Active Time",
        "Geometry Shader Stall Time",
        "# GS threads loaded",
        "Pixel Shader Active Time",
        "Pixel Shader Stall Time",
        "# PS threads loaded",
        "HIZ Fast Z Test Pixels Passing",
        "HIZ Fast Z Test Pixels Failing",
        "Slow Z Test Pixels Failing",
        "Pixel Kill Count",
        "Alpha Test Pixels Failed",
        "Post PS Stencil Pixels Failed",
        "Post PS Z buffer Pixels Failed",
        "3D/GPGPU Render Target Writes",
        "Render Engine Busy",
        "VS bottleneck",
        "GS bottleneck",
};

#define MAX_OA_COUNTERS 60
static uint32_t oa_report_offsets[MAX_OA_COUNTERS];
static int n_oa_report_offsets;

#define MAX_COUNTERS 512
struct counter {
        const char *name;
        int report_offset;
        int config;
        void (*read)(struct counter *self,
                     struct oa_sample *sample_prev,
                     struct oa_sample *sample,
                     struct value *value);
        uint64_t max;
        bool print;
        int pid;
        int tid;
} counters[MAX_COUNTERS];


static int n_counters = N_COMMON_COUNTERS;

static int eu_count;

static uint64_t
read_counter(int id, struct oa_sample *sample)
{
        struct counter *counter = &counters[id];
        struct value value;

        counter->read(counter, NULL /* prev */, sample, &value);

        return value.value;
}

static void
read_oa_counter_raw_cb(struct counter *counter,
                       struct oa_sample *sample_prev,
                       struct oa_sample *sample,
                       struct value *value)
{
        value->timestamp = read_counter(GPU_TIMESTAMP, sample);
        value->value = sample->report[counter->report_offset];
}

static void
read_oa_timestamp_cb(struct counter *counter,
                     struct oa_sample *sample_prev,
                     struct oa_sample *sample,
                     struct value *value)
{
        uint32_t time0 = sample->report[counter->report_offset];
        uint32_t time1 = sample->report[counter->report_offset + 1];
        uint64_t timestamp = (uint64_t)time1 << 32 | time0;

        /* The least significant timestamp bit represents 80ns on Haswell */
        timestamp *= 80;
        timestamp /= 1000; /* usecs */

        value->timestamp = timestamp;
        value->value = timestamp;
}

static struct counter *
add_raw_oa_counter(enum common_counter_id id, int report_offset)
{
        struct counter *counter;

        assert(n_oa_report_offsets < MAX_OA_COUNTERS);

        if (id == EXTRA_COUNTER){
                counter = &counters[n_counters++];
                counter->name = NULL;
        } else {
                counter = &counters[id];
                counter->name = common_counter_names[id];
        }

        counter->report_offset = report_offset;
        counter->read = read_oa_counter_raw_cb;
        counter->max = UINT64_MAX;
        counter->print = false;

        oa_report_offsets[n_oa_report_offsets++] = report_offset;

        return counter;
}

static uint32_t
get_gpu_clock_delta(struct oa_sample *sample_prev, struct oa_sample *sample)
{
        struct counter *clock_counter = &counters[GPU_CORE_CLOCK];
        struct value value0;
        struct value value1;

        clock_counter->read(clock_counter, NULL, sample_prev, &value0);
        clock_counter->read(clock_counter, NULL, sample, &value1);

        /* XXX: note calculating the delta for BDW considering possible
         * wrap around will be a bit more fiddly with its 40bit counters. */
        return (uint32_t)value1.value - (uint32_t)value0.value;
}

static void
read_oa_counter_normalized_by_gpu_duration_cb(struct counter *counter,
                                              struct oa_sample *sample_prev,
                                              struct oa_sample *sample,
                                              struct value *value)
{
        uint32_t value0 = sample_prev->report[counter->report_offset];
        uint32_t value1 = sample->report[counter->report_offset];
        uint32_t delta = value1 - value0;
        uint32_t clk_delta = get_gpu_clock_delta(sample_prev, sample);
        uint64_t timestamp_prev = read_counter(GPU_TIMESTAMP, sample_prev);
        uint64_t timestamp = read_counter(GPU_TIMESTAMP, sample);

        value->timestamp = timestamp_prev + (timestamp - timestamp_prev) / 2;

        if (!clk_delta) {
                value->value = 0;
                return;
        }

        value->value = (uint64_t)delta * 100 / clk_delta;
}

static struct counter *
add_oa_counter_normalised_by_gpu_duration(const char *name,
                                          enum common_counter_id id,
                                          int report_offset)
{
        struct counter *counter = add_raw_oa_counter(id, report_offset);

        counter->name = name;
        counter->read = read_oa_counter_normalized_by_gpu_duration_cb;
        counter->max = 100;

        return counter;
}

static void
read_oa_counter_normalized_by_eu_duration_cb(struct counter *counter,
                                             struct oa_sample *sample_prev,
                                             struct oa_sample *sample,
                                             struct value *value)
{
        uint32_t value0 = sample_prev->report[counter->report_offset];
        uint32_t value1 = sample->report[counter->report_offset];
        uint32_t delta = value1 - value0;
        uint32_t clk_delta = get_gpu_clock_delta(sample_prev, sample);
        uint64_t timestamp_prev = read_counter(GPU_TIMESTAMP, sample_prev);
        uint64_t timestamp = read_counter(GPU_TIMESTAMP, sample);

        delta /= eu_count;

        value->timestamp = timestamp_prev + (timestamp - timestamp_prev) / 2;

        if (!clk_delta) {
                value->value = 0;
                return;
        }

        value->value = (uint64_t)delta * 100 / clk_delta;
}

static struct counter *
add_oa_counter_normalised_by_eu_duration(const char *name,
                                         enum common_counter_id id,
                                         int report_offset)
{
        struct counter *counter = add_raw_oa_counter(id, report_offset);

        counter->name = name;
        counter->read = read_oa_counter_normalized_by_eu_duration_cb;
        counter->max = 100;

        return counter;
}

static void
read_av_thread_cycles_counter_cb (struct counter *counter,
                                  struct oa_sample *sample_prev,
                                  struct oa_sample *sample,
                                  struct value *value)
{
        uint64_t timestamp_prev = read_counter(GPU_TIMESTAMP, sample_prev);
        uint64_t timestamp = read_counter(GPU_TIMESTAMP, sample);
        uint32_t value0 = sample_prev->report[counter->report_offset];
        uint32_t value1 = sample->report[counter->report_offset];
        uint32_t delta = value1 - value0;
        uint32_t n_threads0 = sample_prev->report[counter->config];
        uint32_t n_threads1 = sample->report[counter->config];
        uint32_t spawned = n_threads1 - n_threads0;

        value->timestamp = timestamp_prev + (timestamp - timestamp_prev) / 2;

        if (!spawned) {
                value->value = 0;
                return;
        }

        value->value = delta / spawned;
}

static struct counter *
add_average_thread_cycles_oa_counter (const char *name,
                                      int count_report_offset,
                                      int denominator_report_offset)
{
        struct counter *counter = add_raw_oa_counter(EXTRA_COUNTER,
                                                     count_report_offset);

        counter->name = name;
        counter->read = read_av_thread_cycles_counter_cb;
        counter->config = denominator_report_offset;

        return counter;
}

static void
init_hsw_oa_counters(void)
{
        struct counter *c;
        int a_offset = 3; /* A0 */
        int b_offset = a_offset + 45; /* B0 */

        c = add_raw_oa_counter(GPU_TIMESTAMP, 1);
        c->read = read_oa_timestamp_cb;
        c->print = true;

        add_raw_oa_counter(AGGREGATE_CORE_ARRAYS_ACTIVE, a_offset);
        c = add_oa_counter_normalised_by_eu_duration("EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset);
        c->max = 100;
        c->print = true;
        add_raw_oa_counter(AGGREGATE_CORE_ARRAYS_STALLED, a_offset + 1);
        c = add_oa_counter_normalised_by_eu_duration("EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 1);
        c->print = true;
        add_raw_oa_counter(VS_ACTIVE_TIME, a_offset + 2);
        c = add_oa_counter_normalised_by_eu_duration("VS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 2);
        c->print = true;
        add_raw_oa_counter(VS_STALL_TIME, a_offset + 3);
        c = add_oa_counter_normalised_by_eu_duration("VS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 3);
        c->print = true;
        add_raw_oa_counter(NUM_VS_THREADS_LOADED, a_offset + 5);

        c = add_average_thread_cycles_oa_counter("Av. cycles per VS thread",
                                                 a_offset + 2,
                                                 a_offset + 5);
        c->print = true;
        c = add_average_thread_cycles_oa_counter("Av. stalled cycles per VS thread",
                                                 a_offset + 3,
                                                 a_offset + 5);
        c->print = true;

        add_raw_oa_counter(HS_ACTIVE_TIME, a_offset + 7);
        c = add_oa_counter_normalised_by_eu_duration("HS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 7);
        c->print = true;
        add_raw_oa_counter(HS_STALL_TIME, a_offset + 8);
        c = add_oa_counter_normalised_by_eu_duration("HS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 8);
        c->print = true;
        add_raw_oa_counter(NUM_HS_THREADS_LOADED, a_offset + 10);
        c = add_average_thread_cycles_oa_counter("Av. cycles per HS thread",
                                                 a_offset + 7,
                                                 a_offset + 10);
        c->print = true;
        c = add_average_thread_cycles_oa_counter("Av. stalled cycles per HS thread",
                                                 a_offset + 8,
                                                 a_offset + 10);
        c->print = true;

        add_raw_oa_counter(DS_ACTIVE_TIME, a_offset + 12);
        c = add_oa_counter_normalised_by_eu_duration("DS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 12);
        c->print = true;
        add_raw_oa_counter(DS_STALL_TIME, a_offset + 13);
        c = add_oa_counter_normalised_by_eu_duration("DS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 13);
        c->print = true;
        add_raw_oa_counter(NUM_DS_THREADS_LOADED, a_offset + 15);
        c = add_average_thread_cycles_oa_counter("Av. cycles per DS thread",
                                                 a_offset + 12,
                                                 a_offset + 15);
        c->print = true;
        c = add_average_thread_cycles_oa_counter("Av. stalled cycles per DS thread",
                                                 a_offset + 13,
                                                 a_offset + 15);
        c->print = true;

        add_raw_oa_counter(CS_ACTIVE_TIME, a_offset + 17);
        c = add_oa_counter_normalised_by_eu_duration("CS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 17);
        c->print = true;
        add_raw_oa_counter(CS_STALL_TIME, a_offset + 18);
        c = add_oa_counter_normalised_by_eu_duration("CS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 18);
        c->print = true;
        add_raw_oa_counter(NUM_CS_THREADS_LOADED, a_offset + 20);
        c = add_average_thread_cycles_oa_counter("Av. cycles per CS thread",
                                                 a_offset + 17,
                                                 a_offset + 20);
        c->print = true;
        c = add_average_thread_cycles_oa_counter("Av. stalled cycles per CS thread",
                                                 a_offset + 18,
                                                 a_offset + 20);
        c->print = true;


        add_raw_oa_counter(GS_ACTIVE_TIME, a_offset + 22);
        c = add_oa_counter_normalised_by_eu_duration("GS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 22);
        c->print = true;
        add_raw_oa_counter(GS_STALL_TIME, a_offset + 23);
        c = add_oa_counter_normalised_by_eu_duration("GS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 23);
        c->print = true;
        add_raw_oa_counter(NUM_GS_THREADS_LOADED, a_offset + 25);
        c = add_average_thread_cycles_oa_counter("Av. cycles per GS thread",
                                                 a_offset + 22,
                                                 a_offset + 25);
        c->print = true;
        c = add_average_thread_cycles_oa_counter("Av. stalled cycles per GS thread",
                                                 a_offset + 23,
                                                 a_offset + 25);
        c->print = true;


        add_raw_oa_counter(PS_ACTIVE_TIME, a_offset + 27);
        c = add_oa_counter_normalised_by_eu_duration("PS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 27);
        c->print = true;
        add_raw_oa_counter(PS_STALL_TIME, a_offset + 28);
        c = add_oa_counter_normalised_by_eu_duration("PS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 28);
        c->print = true;
        add_raw_oa_counter(NUM_PS_THREADS_LOADED, a_offset + 30);
        c = add_average_thread_cycles_oa_counter("Av. cycles per PS thread",
                                                 a_offset + 27,
                                                 a_offset + 30);
        c->print = true;
        c = add_average_thread_cycles_oa_counter("Av. stalled cycles per PS thread",
                                                 a_offset + 28,
                                                 a_offset + 30);
        c->print = true;

        add_raw_oa_counter(HIZ_FAST_Z_PASSING, a_offset + 32);
        add_raw_oa_counter(HIZ_FAST_Z_FAILING, a_offset + 33);

        add_raw_oa_counter(SLOW_Z_FAILING, a_offset + 35);

        /* XXX: caveat: it's 2x real No. when PS has 2 output colors */
        add_raw_oa_counter(PIXEL_KILL_COUNT, a_offset + 36);

        add_raw_oa_counter(ALPHA_TEST_FAILED, a_offset + 37);
        add_raw_oa_counter(POST_PS_STENCIL_TEST_FAILED, a_offset + 38);
        add_raw_oa_counter(POST_PS_Z_TEST_FAILED, a_offset + 39);

        add_raw_oa_counter(RENDER_TARGET_WRITES, a_offset + 40);

        /* XXX: there are several conditions where this doesn't increment... */
        add_raw_oa_counter(RENDER_ENGINE_BUSY, a_offset + 41);
        c = add_oa_counter_normalised_by_gpu_duration("Render Engine Busy %",
                                                      EXTRA_COUNTER,
                                                      a_offset + 41);
        c->print = true;

        add_raw_oa_counter(VS_BOTTLENECK, a_offset + 42);
        add_raw_oa_counter(GS_BOTTLENECK, a_offset + 43);
        c = add_raw_oa_counter(GPU_CORE_CLOCK, b_offset);
        c->print = true;
}

/* XXX: For BDW+ we need to check fuse registers */
static int
get_eu_count(uint32_t devid)
{
        if (IS_HSW_GT1(devid))
                return 10;
        if (IS_HSW_GT2(devid))
                return 20;
        if (IS_HSW_GT3(devid))
                return 40;
        assert(0);
}

static void
init_oa_counters(uint32_t devid)
{
        if (IS_HASWELL(devid))
                init_hsw_oa_counters();
        else {
                fprintf(stderr, "Only Haswell supported currently");
                exit(1);
        }
}

struct rdtsc_t
{
        union {
                struct {
                        uint32_t low;
                        uint32_t hi;
                } split;
                uint64_t full_value;
        } u;
};

static uint64_t
__rdtsc (void)
{
        struct rdtsc_t rdtsc;
        asm ("rdtsc; movl %%edx,%0; movl %%eax,%1"
                        : "=r" (rdtsc.u.split.hi), "=r" (rdtsc.u.split.low)
                        : /* input */
                        : "%edx", "%eax");
        return rdtsc.u.full_value;
}

static uint64_t tsc_hz;
static uint64_t tsc_per_microsecond;

static void
calibrate_tsc_to_hz (void)
{
        uint64_t time0, time1 = 0;
        uint64_t diff;

        while (1) {
                struct timespec delay;
                /* Determine the frequency of our time source */
                time0 = __rdtsc ();
                delay.tv_sec = 0;
                delay.tv_nsec = 1000000000/4;
                if (nanosleep (&delay, NULL) == -1) {
                        if (errno == EINTR)
                                continue;
                        else {
                                fprintf (stderr, "Failed to calibrate tsc hz: %m");
                                exit (1);
                        }
                } else {
                        time1 = __rdtsc ();
                        break;
                }
        }

        assert (time1 > time0);
        diff = time1 - time0;
        tsc_hz = diff * 4;

        tsc_per_microsecond = tsc_hz / 1000000;

        //printf ("tsc hz=%llu\n", tsc_hz);
        //exit (1);
}

static uint64_t
event_timestamp_to_tsc (struct event *event, uint64_t timestamp)
{
        uint16_t time_shift = event->mmap_page->time_shift;
        uint32_t time_mult = event->mmap_page->time_mult;
        uint64_t time_zero = event->mmap_page->time_zero;

        uint64_t time = timestamp - time_zero;
        uint64_t quot = time / time_mult;
        uint64_t rem = time % time_mult;

        return (quot << time_shift) + (rem << time_shift) / time_mult;
}

static uint64_t
read_file_uint64 (const char *file)
{
        char buf[32];
        int fd, n;

        fd = open (file, 0);
        if (fd < 0)
                return 0;
        n = read (fd, buf, sizeof (buf) - 1);
        close (fd);
        if (n < 0)
                return 0;

        buf[n] = '\0';
        return strtoull (buf, 0, 0);
}

static uint64_t
lookup_i915_oa_id (void)
{
        return read_file_uint64 ("/sys/bus/event_source/devices/i915_oa/type");
}

static long
perf_event_open(struct perf_event_attr *hw_event,
                pid_t pid,
                int cpu,
                int group_fd,
                unsigned long flags)
{
        return syscall (__NR_perf_event_open, hw_event, pid, cpu, group_fd, flags);
}

static struct event *
open_i915_oa_event(uint64_t report_format,
                   int period_exponent,
                   bool single_context,
                   int drm_fd,
                   uint32_t ctx_id)
{
        struct perf_event_attr attr;
        unsigned int mmap_size = 32 * page_size; /* NB: must be power of two */
        int event_fd;
        void *mmap_base;
        struct perf_event_mmap_page *mmap_page;
        struct event *event;
        struct epoll_event ev = { 0 };
        int ret;

        memset(&attr, 0, sizeof (struct perf_event_attr));
        attr.size = sizeof (struct perf_event_attr);
        attr.type = lookup_i915_oa_id();

        attr.config |= report_format;
        attr.config |= (uint64_t)period_exponent << I915_PERF_OA_TIMER_EXPONENT_SHIFT;

        if (single_context) {
                attr.config |= I915_PERF_OA_SINGLE_CONTEXT_ENABLE;
                attr.config |= ctx_id & I915_PERF_OA_CTX_ID_MASK;
                attr.config1 = drm_fd;
        }

        attr.sample_type = PERF_SAMPLE_TIME | PERF_SAMPLE_READ | PERF_SAMPLE_RAW;
        attr.disabled = 1;
        attr.sample_period = 1;
        attr.watermark = true;
        attr.wakeup_watermark = mmap_size / 4;

        /* XXX: Notes about pid/cpu choice:
         *
         * The combination of pid = -1, cpu = -1 which is conceptually
         * what we want is not supported in perf and it looks like it
         * could have wide reaching consequences on its design to try
         * and enable this.
         *
         * I considered subverting the semantics of pid = 0, cpu = -1
         * since that combination doesn't require root privileges but
         * but in that case perf will automatically enable and disable
         * our pmu as the current process gets scheduled and that
         * would be quite complex to change.
         *
         * The combination pid = -1, cpu = 0 requires CAP_SYS_ADMIN
         * normally but our pmu patch introduces the concept of a
         * device based pmu whereby events/core.c now understands that
         * our pmu doesn't relate to the cpu and skips the
         * CAP_SYS_ADMIN check because this combination no longer
         * implies that a process is able to profile the running of
         * another process, and it is instead our responsibility to
         * enforce any appropriate permission checks.
         * */
        event_fd = perf_event_open(&attr,
                        -1,  /* pid */
                        0, /* cpu */
                        -1, /* group fd */
                        PERF_FLAG_FD_CLOEXEC); /* flags */
        if (event_fd == -1) {
                fprintf(stderr, "Error opening event %llx: %m\n", attr.config);
                return NULL;
        }

        /* NB: A read-only mapping will result in overwrite semantics whereby the kernel
         * won't stop writing data when the buffer is full, and will instead overwrite
         * old data. */
        mmap_base = mmap(NULL, mmap_size + page_size,
                         PROT_READ | PROT_WRITE, MAP_SHARED, event_fd, 0);
        if (mmap_base == MAP_FAILED) {
                fprintf (stderr, "Error mapping circular buffer, %m\n");
                close (event_fd);
                return NULL;
        }

        mmap_page = mmap_base;

        if (!mmap_page->cap_user_time) {
                fprintf(stderr, "System doesn't support a constant, non-stop "
                                "timestamp counter\n");
                exit(1);
        }
        if (!mmap_page->cap_user_time_zero) {
                fprintf(stderr, "Kernel hasn't provided a TSC base so we can't "
                                "correlate perf timestamps with the TSC\n");
                exit(1);
        }

        ret = ioctl(event_fd, PERF_EVENT_IOC_ENABLE, 0);
        if (ret < 0) {
                fprintf(stderr, "Error enabling event: %m\n");
                exit(1);
        }

        fprintf(stderr, "i915 event: format = %" PRIu64 ", ctx = %d:\n", report_format, ctx_id);
        fprintf(stderr, "  mmap info:\n");
        fprintf(stderr, "  base = %p\n", mmap_base);
        fprintf(stderr, "  version = %u\n", mmap_page->version);
        fprintf(stderr, "  compat version = %u\n", mmap_page->compat_version);
        fprintf(stderr, "  head = %" PRIu64 "\n", (uint64_t)mmap_page->data_head);
        fprintf(stderr, "  tail = %" PRIu64 "\n", (uint64_t)mmap_page->data_tail);

        event = malloc(sizeof (struct event));
        event->cpu = -1;
        event->fd = event_fd;
        event->mmap_size = mmap_size;
        event->mmap_base = mmap_base;
        event->mmap_page = mmap_page;

        sample_array_init(&event->array);

        events[n_events++] = event;

        ev.data.fd = event_fd;
        ev.events = EPOLLIN;
        epoll_ctl (epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev);

        return event;
}

static void
record_oa_sample(struct event *event,
                 const struct perf_event_header *header)
{
        struct oa_perf_sample *perf_sample = (struct oa_perf_sample *)header;
        struct oa_sample *sample = sample_array_append(&event->array);

        //fprintf(stderr, "len = %d\n", event->array.len);

        assert(header->type == PERF_RECORD_SAMPLE);
        assert((sizeof(sample->report) + 4)== perf_sample->raw_size);

        //fprintf(stderr, "Recording scalar sample for event = %p: value = %" PRIu64 ", time = %" PRIu64 "\n",
        //        event, perf_sample->value, perf_sample->time);
        sample->log_timestamp = perf_sample->time;
        sample->nominal_value = perf_sample->value;
        memcpy(&sample->report, perf_sample->raw_data, perf_sample->raw_size);

#if 0
        fprintf(stderr, "report id = %" PRIu32 "\n", sample->report.report_id);
        fprintf(stderr, "timer0 = %" PRIu32 " timer1 = %" PRIu32 "\n", sample->report.timestamp0, sample->report.timestamp1);
        fprintf(stderr, "A0 = %" PRIu32 "\n", sample->report.a0);
        fprintf(stderr, "A1 = %" PRIu32 "\n", sample->report.a1);
        fprintf(stderr, "B0 = %" PRIu32 "\n", sample->report.b0);

        for (int i = 0; i < 10; i++) {
                fprintf(stderr, "  %d) %" PRIu32 "\n", i, ((uint32_t *)(&sample->report))[i]);
        }
#endif
}

static void
read_event_samples (struct event *event, bool flush)
{
        uint8_t *data = event->mmap_base + page_size;
        const unsigned int size = event->mmap_size;
        const uint64_t mask = size - 1;
        uint64_t head;
        uint64_t tail;
        //uint64_t value;

        if (flush) {
                uint64_t count;

                /* A well defined side effect of reading the sample count of
                 * an i915 OA event is that all outstanding counter reports
                 * will be flushed into the perf mmap buffer... */
                read(event->fd, &count, 8);
        }

        head = event->mmap_page->data_head;
        rmb ();

        tail = event->mmap_page->data_tail;

        //fprintf(stderr, "Handle event mask = 0x%" PRIx64
        //        " head=%" PRIu64 " tail=%" PRIu64 "\n", mask, head, tail);

        while (TAKEN(head, tail, size)) {
                const struct perf_event_header *header =
                        (const struct perf_event_header *)(data + (tail & mask));

                if (header->size == 0) {
                        fprintf(stderr, "Spurious header size == 0\n");
                        exit(1);
                }

                if (header->size > (head - tail)) {
                        fprintf(stderr, "Spurious header size would overshoot head\n");
                        exit(1);
                }

                //fprintf(stderr, "header = %p tail=%" PRIu64 " size=%d\n",
                //        header, tail, header->size);

                if ((const uint8_t *)header + header->size > data + size) {
                        int before;

                        if (header->size > scratch_buffer_size) {
                                uint8_t *b = realloc(scratch_buffer, header->size);
                                if (b == NULL) {
                                        fprintf (stderr, "Failed to allocate "
                                                 "scratch buffer for split sample\n");
                                        exit (1);
                                }

                                scratch_buffer = b;
                                scratch_buffer_size = header->size;
                        }

                        before = data + size - (const uint8_t *)header;

                        memcpy(scratch_buffer, header, before);
                        memcpy(scratch_buffer + before, data, header->size - before);

                        header = (struct perf_event_header *)scratch_buffer;
                        //fprintf(stderr, "DEBUG: split\n");
                        //exit(1);
                }

                switch (header->type) {
                case PERF_RECORD_LOST: {
                       struct {
                               struct perf_event_header header;
                               uint64_t id;
                               uint64_t n_lost;
                       } *lost = (void *)header;
                       fprintf(stderr, "Warning: Lost %" PRIu64 " events\n", lost->n_lost);
                       break;
                }
                case PERF_RECORD_THROTTLE:
                       fprintf(stderr, "Warning: Sampling has been throttled\n");
                       break;
                case PERF_RECORD_UNTHROTTLE:
                       fprintf(stderr, "Warning: Sampling has been unthrottled\n");
                       break;
                case PERF_RECORD_SAMPLE:
                       record_oa_sample(event, header);
                       break;
                default:
                       fprintf(stderr, "Spurious header type = %d\n", header->type);
                }

                //fprintf(stderr, "Tail += %d\n", header->size);

                tail += header->size;
        }

        event->mmap_page->data_tail = tail;
        wmb();

#if 0
        printf ("cpu %d:\n", event->cpu);
        printf ("  head = %" PRIu64 "\n", head);
        printf ("  tail = %" PRIu64 "\n", tail);
        printf ("  taken = %u\n", (unsigned int)TAKEN (head, tail, size));
        printf ("  remaining = %u\n", (unsigned int)REMAINING (head, tail, size));

        read (event->fd, &value, 8);
        printf ("  Event read = %" PRIu64 "\n", value);
#endif
}

static void
read_perf_samples (bool flush)
{
        //fprintf(stderr, "Recording samples...\n");

        for (int i = 0; i < n_events; i++)
                read_event_samples (events[i], flush);
}

static void
perf_timer_cb (void)
{
        read_perf_samples (false /* don't force flush */);
}

static void
print_clock(const char *name, int clock) {
	if (clock == -1)
		printf("%s clock: unknown", name);
	else
		printf("%s clock: %d Mhz", name, clock);
}

static int
print_clock_info(void)
{
        int max_render_clock;
        int cur_render_clock;

        max_render_clock = read_file_uint64("/sys/class/drm/card0/gt_max_freq_mhz");
        cur_render_clock = read_file_uint64("/sys/class/drm/card0/gt_cur_freq_mhz");

        print_clock("max render", max_render_clock);
        printf("  ");
        print_clock("current render", cur_render_clock);

	printf("\n");
	return -1;
}

#define PERCENTAGE_BAR_END	(79)

static const char *bars[] = {
	" ",
	"▏",
	"▎",
	"▍",
	"▌",
	"▋",
	"▊",
	"▉",
	"█"
};

static void
print_percentage_bar(float percent, int cur_line_len)
{
	int bar_avail_len = (PERCENTAGE_BAR_END - cur_line_len - 1) * 8;
	int bar_len = bar_avail_len * (percent + .5) / 100.0;
	int i;

	for (i = bar_len; i >= 8; i -= 8) {
		printf("%s", bars[8]);
		cur_line_len++;
	}
	if (i) {
		printf("%s", bars[i]);
		cur_line_len++;
	}

	/* NB: We can't use a field width with utf8 so we manually
	* guarantee a field with of 45 chars for any bar. */
	printf("%*s", PERCENTAGE_BAR_END - cur_line_len, "");
}

static void
print_timer_cb (void)
{
        struct event *event = i915_oa_event;
        struct oa_sample *samples;
        int n_samples;
        unsigned short int max_lines;
        struct winsize ws;
        char clear_screen[] = {0x1b, '[', 'H',
                0x1b, '[', 'J',
                0x0};

        read_perf_samples (true /* force flush */);

        samples = event->array.samples;

        /* Limit the number of lines printed to the terminal height so the
         * most important info (at the top) will stay on screen. */
        max_lines = -1;
        if (ioctl(0, TIOCGWINSZ, &ws) != -1)
                max_lines = ws.ws_row - 6; /* exclude header lines */
        if (max_lines > n_counters)
                max_lines = n_counters;

        printf("%s", clear_screen);
        print_clock_info();

        n_samples = event->array.len;
        if (n_samples <= 2) {
                fprintf(stderr, "No counter samples available\n");
                return;
        }

        printf("\n%30s  %s\n", "counter", "value");
        printf("%40s       0%%                          100%%\n", "");
        printf("%40s       ┌──────────────────────────────┐\n", "");
        for (int i = 0, lines = 0; i < n_counters && lines < max_lines; i++) {
                struct counter *counter = &counters[i];
                struct oa_sample *sample = samples + n_samples - 1;
                struct oa_sample *prev_sample = samples + n_samples - 2;
                struct value value;

                if (!counter->print)
                        continue;

                counter->read(counter, prev_sample, sample, &value);

                if (value.value > counter->max) {
                        fprintf(stderr, "Warning counter overflow for %s "
                                        "(max = %" PRIu64 ", value = %" PRIu64 "\n",
                                        counter->name,
                                        counter->max,
                                        value.value);
                        value.value = counter->max;
                }

                if (counter->max == 100) {
                        int len = printf("%40s: %3d%%: ", counter->name, (int)value.value);
                        print_percentage_bar (value.value, len);
                        printf("\n");
                } else
                        printf("%40s:   %" PRIu64 "\n", counter->name, value.value);

                lines++;
        }

        event->array.len = 0;
}

static uint32_t read_device_param(int id, const char *param)
{
        char *name;
        int ret = asprintf(&name, "/sys/class/drm/renderD%u/"
                           "device/%s", id, param);
        uint32_t value;

        igt_assert(ret != -1);

        value = read_file_uint64(name);
        free(name);

        return value;
}

static int open_render_node(struct intel_device *dev)
{
        char *name;
        int i, fd;

        for (i = 128; i < (128 + 16); i++) {
                int ret;

                ret = asprintf(&name, "/dev/dri/renderD%u", i);
                igt_assert(ret != -1);

                fd = open(name, O_RDWR);
                free(name);

                if (fd == -1)
                        continue;

                if (read_device_param(i, "vendor") != 0x8086) {
                        close(fd);
                        fd = -1;
                        continue;
                }

                dev->device = read_device_param(i, "device");
                dev->subsystem_device = read_device_param(i, "subsystem_device");
                dev->subsystem_vendor = read_device_param(i, "subsystem_vendor");

                return fd;
        }

        return fd;
}

static void scratch_bo_init(drm_intel_bufmgr *bufmgr,
                            struct igt_buf *buf,
			    int width, int height, int stride)
{
	drm_intel_bo *bo;

	bo = drm_intel_bo_alloc(bufmgr, "", SCRATCH_BO_SIZE, 4096);

	buf->bo = bo;
	buf->stride = stride;
	buf->tiling = I915_TILING_NONE;
	buf->size = SCRATCH_BO_SIZE;
}

static int create_timer(int milliseconds)
{
        int timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC | TFD_NONBLOCK);
        struct itimerspec period;
        struct epoll_event ev = { 0 };

        period.it_value.tv_sec = milliseconds / 1000;
        period.it_value.tv_nsec = (milliseconds % 1000) * 1000000;
        period.it_interval.tv_sec = milliseconds / 1000;
        period.it_interval.tv_nsec = (milliseconds % 1000) * 1000000;

        timerfd_settime(timer_fd, 0, &period, NULL);

        ev.data.fd = timer_fd;
        ev.events = EPOLLIN;
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_fd, &ev);

        return timer_fd;
}

int
main (int argc, char **argv)
{
        sigset_t mask;
        struct epoll_event ev = { 0 };
        int perf_timer_fd;
        int print_timer_fd;
        int drm_fd;
	drm_intel_context *context;
	drm_intel_bufmgr *bufmgr;
        int period_exponent;
	//igt_render_copyfunc_t render_copy = NULL;
	//struct intel_batchbuffer *batch = NULL;
	//struct igt_buf src, dst;

        drm_fd = open_render_node(&intel_dev);
        if (drm_fd < 0) {
                fprintf(stderr, "Failed to open render node");
                exit(1);
        }

        eu_count = get_eu_count(intel_dev.device);
        init_oa_counters(intel_dev.device);

	bufmgr = drm_intel_bufmgr_gem_init(drm_fd, 4096);
	context = drm_intel_gem_context_create(bufmgr);
	if (!context) {
                fprintf(stderr, "Failed to create GEM context\n");
                exit(1);
        }

        fprintf(stderr, "GEM Context ID = %u\n", context->ctx_id);

        //n_cpus = sysconf(_SC_NPROCESSORS_ONLN);
        page_size = sysconf(_SC_PAGE_SIZE);

#if 0
        render_copy = igt_get_render_copyfunc(intel_dev.device);

        batch = intel_batchbuffer_alloc(bufmgr, intel_dev.device);

	scratch_bo_init(bufmgr, &src, SCRATCH_BO_WIDTH, SCRATCH_BO_HEIGHT, SCRATCH_BO_STRIDE);
	scratch_bo_init(bufmgr, &dst, SCRATCH_BO_WIDTH, SCRATCH_BO_HEIGHT, SCRATCH_BO_STRIDE);
#endif

        scratch_buffer_size = MAX_REPORT_SIZE + sizeof (struct oa_perf_sample);
        scratch_buffer = malloc(scratch_buffer_size);

        calibrate_tsc_to_hz();

        epoll_fd = epoll_create1(EPOLL_CLOEXEC);

        sigemptyset(&mask);
        sigaddset(&mask, SIGINT);
        sigprocmask(SIG_BLOCK, &mask, NULL);

        sigint_fd = signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);

        ev.data.fd = sigint_fd;
        ev.events = EPOLLIN;
        epoll_ctl (epoll_fd, EPOLL_CTL_ADD, sigint_fd, &ev);

        perf_timer_fd = create_timer(10 /* milliseconds */);
        print_timer_fd = create_timer(1000 /* milliseconds */);

	/* The timestamp for HSW+ increments every 80ns
	 *
	 * The period_exponent gives a sampling period as follows:
	 *   sample_period = 80ns * 2^(period_exponent + 1)
	 *
	 * FIXME: we need to choose a short enough period to catch
	 * counters wrapping.
	 *
	 * The overflow period for Haswell can be calculated as:
	 *
	 * 2^32 / (n_eus * max_gen_freq * 2)
	 * (E.g. 40 EUs @ 1GHz = ~53ms)
	 *
	 * Currently we just sample ~ every 5 milliseconds...
	 */
        period_exponent = 15;

#if 1
        i915_oa_event = open_i915_oa_event(I915_PERF_OA_FORMAT_A45_B8_C8_HSW,
                                           period_exponent,
                                           false, /* profile cross-context */
                                           -1, /* drm fd ignored */
                                           0); /* ctx-id ignored */
#else
        i915_oa_event = open_i915_oa_event(I915_PERF_OA_FORMAT_A45_B8_C8_HSW,
                                           period_exponent,
                                           true, /* profile single context */
                                           drm_fd,
                                           context->ctx_id);
#endif
        if (!i915_oa_event) {
                fprintf (stderr, "Failed to open i915 OA event\n");
                return EXIT_FAILURE;
        }

        for (;;) {
                struct epoll_event poll_events[256];
                int n = epoll_wait(epoll_fd, poll_events, 256, -1);

                /* XXX:
                 * Debug workload to test profiling a specific context
                 * without root permissions...  */
#if 0
                for (i = 0; i < 10000; i++)
                        render_copy(batch, context,
                                    &src, 0, 0, SCRATCH_BO_WIDTH, SCRATCH_BO_HEIGHT,
                                    &dst, SCRATCH_BO_WIDTH / 2, SCRATCH_BO_HEIGHT / 2);
#endif

                for (int i = 0; i < n; i++) {
                        if (poll_events[i].data.fd == sigint_fd)
                                goto quit;
                        else if (poll_events[i].data.fd == perf_timer_fd) {
                                uint64_t ticks;
                                read (perf_timer_fd, &ticks, 8);
                                perf_timer_cb();
                        } else if (poll_events[i].data.fd == print_timer_fd) {
                                uint64_t ticks;
                                read (print_timer_fd, &ticks, 8);
                                print_timer_cb();
                        } else
                                read_perf_samples (false);
                }
        }

quit:
        fprintf (stderr, "Quit\n");

        return 0;
}
