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
#include <getopt.h>
#include <math.h>

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
#define SAMPLE_BUF_SIZE (16 * 1024 * 1024)

#define SAMPLES_TID         1234
#define CS_TID              (SAMPLES_TID + 100)
#define STATS_TID_BASE      (CS_TID + 100)
#define STATS_COUNT         9
#define INSTDONE_TID_BASE   (STATS_TID_BASE + STATS_COUNT)
#define FREQ_TID            (INSTDONE_TID_BASE + 200)
#define OA_TID              (FREQ_TID + 10)

#define SCRATCH_BO_WIDTH 512
#define SCRATCH_BO_STRIDE (SCRATCH_BO_WIDTH*4)
#define SCRATCH_BO_HEIGHT 512
#define SCRATCH_BO_SIZE (SCRATCH_BO_HEIGHT * SCRATCH_BO_STRIDE)

/* XXX: HACK */
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

struct event {

        int cpu;

        int fd;

        union {
                struct perf_event_mmap_page *mmap_page;
                uint8_t *mmap_base;
        };
        unsigned int mmap_size;

        void (*record_sample)(struct event *event,
                              const struct perf_event_header *header);
        uint8_t *samples;
        unsigned samples_offset;
};

struct scalar_sample {
        uint64_t time;
        uint64_t value;
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

#define HSW_MAX_REPORT_LEN 64

struct oa_sample {
        uint64_t log_timestamp; /* time OA report was forwarded to userspace */
        uint64_t nominal_value;
        uint32_t report[HSW_MAX_REPORT_LEN];
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
        void (*read)(struct counter *self,
                     struct oa_sample *sample_prev,
                     struct oa_sample *sample,
                     struct value *value);
        uint64_t max;
        int pid;
        int tid;
} counters[MAX_COUNTERS];


static int n_counters = N_COMMON_COUNTERS;

static int eu_count;

static size_t
report_format_size(uint64_t format)
{
        switch(format) {
        case I915_PERF_OA_FORMAT_A13_HSW:
                return 64;
        case I915_PERF_OA_FORMAT_A29_HSW:
                return 128;
        case I915_PERF_OA_FORMAT_A13_B8_C8_HSW:
                return 128;
        case I915_PERF_OA_FORMAT_A29_B8_C8_HSW:
                return 92;
        case I915_PERF_OA_FORMAT_B4_C8_HSW:
                return 64;
        case I915_PERF_OA_FORMAT_A45_B8_C8_HSW:
                return 256;
        case I915_PERF_OA_FORMAT_B4_C8_A16_HSW:
                return 128;
        case I915_PERF_OA_FORMAT_C4_B8_HSW:
                return 64;
        default:
                assert(0);
        }
}

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
add_oa_counter_normalised_by_gpu_duration(const char *name,
                                          enum common_counter_id id,
                                          int report_offset)
{
        struct counter *counter = add_raw_oa_counter(id, report_offset);

        counter->name = name;
        counter->read = read_oa_counter_normalized_by_gpu_duration_cb;

        return counter;
}

static struct counter *
add_oa_counter_normalised_by_eu_duration(const char *name,
                                         enum common_counter_id id,
                                         int report_offset)
{
        struct counter *counter = add_raw_oa_counter(id, report_offset);

        counter->name = name;
        counter->read = read_oa_counter_normalized_by_eu_duration_cb;

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

        add_raw_oa_counter(AGGREGATE_CORE_ARRAYS_ACTIVE, a_offset);
        c = add_oa_counter_normalised_by_eu_duration("EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset);
        c->max = 100;
        add_raw_oa_counter(AGGREGATE_CORE_ARRAYS_STALLED, a_offset + 1);
        c = add_oa_counter_normalised_by_eu_duration("EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 1);
        c->max = 100;
        add_raw_oa_counter(VS_ACTIVE_TIME, a_offset + 2);
        c = add_oa_counter_normalised_by_eu_duration("VS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 2);
        c->max = 100;
        add_raw_oa_counter(VS_STALL_TIME, a_offset + 3);
        c = add_oa_counter_normalised_by_eu_duration("VS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 3);
        c->max = 100;
        add_raw_oa_counter(NUM_VS_THREADS_LOADED, a_offset + 5);
        add_raw_oa_counter(HS_ACTIVE_TIME, a_offset + 7);
        c = add_oa_counter_normalised_by_eu_duration("HS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 7);
        c->max = 100;
        add_raw_oa_counter(HS_STALL_TIME, a_offset + 8);
        c = add_oa_counter_normalised_by_eu_duration("HS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 8);
        c->max = 100;
        add_raw_oa_counter(NUM_HS_THREADS_LOADED, a_offset + 10);
        add_raw_oa_counter(DS_ACTIVE_TIME, a_offset + 12);
        c = add_oa_counter_normalised_by_eu_duration("DS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 12);
        c->max = 100;
        add_raw_oa_counter(DS_STALL_TIME, a_offset + 13);
        c = add_oa_counter_normalised_by_eu_duration("DS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 13);
        c->max = 100;
        add_raw_oa_counter(NUM_DS_THREADS_LOADED, a_offset + 15);
        add_raw_oa_counter(CS_ACTIVE_TIME, a_offset + 17);
        c = add_oa_counter_normalised_by_eu_duration("CS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 17);
        c->max = 100;
        add_raw_oa_counter(CS_STALL_TIME, a_offset + 18);
        c = add_oa_counter_normalised_by_eu_duration("CS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 18);
        c->max = 100;
        add_raw_oa_counter(NUM_CS_THREADS_LOADED, a_offset + 20);
        add_raw_oa_counter(GS_ACTIVE_TIME, a_offset + 22);
        c = add_oa_counter_normalised_by_eu_duration("GS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 22);
        c->max = 100;
        add_raw_oa_counter(GS_STALL_TIME, a_offset + 23);
        c = add_oa_counter_normalised_by_eu_duration("GS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 23);
        c->max = 100;
        add_raw_oa_counter(NUM_GS_THREADS_LOADED, a_offset + 25);
        add_raw_oa_counter(PS_ACTIVE_TIME, a_offset + 27);
        c = add_oa_counter_normalised_by_eu_duration("PS EU Active %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 27);
        c->max = 100;
        add_raw_oa_counter(PS_STALL_TIME, a_offset + 28);
        c = add_oa_counter_normalised_by_eu_duration("PS EU Stalled %",
                                                     EXTRA_COUNTER,
                                                     a_offset + 28);
        c->max = 100;
        add_raw_oa_counter(NUM_PS_THREADS_LOADED, a_offset + 30);
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
        c->max = 100;

        add_raw_oa_counter(VS_BOTTLENECK, a_offset + 42);
        add_raw_oa_counter(GS_BOTTLENECK, a_offset + 43);
        add_raw_oa_counter(GPU_CORE_CLOCK, b_offset);
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
__rdtsc(void)
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
calibrate_tsc_to_hz(void)
{
        uint64_t time0, time1 = 0;
        uint64_t diff;

        while (1) {
                struct timespec delay;
                /* Determine the frequency of our time source */
                time0 = __rdtsc();
                delay.tv_sec = 0;
                delay.tv_nsec = 1000000000/4;
                if (nanosleep(&delay, NULL) == -1) {
                        if (errno == EINTR)
                                continue;
                        else {
                                fprintf(stderr, "Failed to calibrate tsc hz: %m");
                                exit(1);
                        }
                } else {
                        time1 = __rdtsc();
                        break;
                }
        }

        assert(time1 > time0);
        diff = time1 - time0;
        tsc_hz = diff * 4;

        tsc_per_microsecond = tsc_hz / 1000000;

        //printf("tsc hz=%llu\n", tsc_hz);
        //exit(1);
}

static uint64_t
event_timestamp_to_tsc(struct event *event, uint64_t timestamp)
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
read_file_uint64(const char *file)
{
        char buf[32];
        int fd, n;

        fd = open(file, 0);
        if (fd < 0)
                return 0;
        n = read(fd, buf, sizeof (buf) - 1);
        close(fd);
        if (n < 0)
                return 0;

        buf[n] = '\0';
        return strtoull(buf, 0, 0);
}

static uint64_t
lookup_i915_oa_id(void)
{
        return read_file_uint64("/sys/bus/event_source/devices/i915_oa/type");
}

static long
perf_event_open(struct perf_event_attr *hw_event,
                 pid_t pid,
                 int cpu,
                 int group_fd,
                 unsigned long flags)
{
        return syscall(__NR_perf_event_open, hw_event, pid, cpu, group_fd, flags);
}

static struct event *
open_i915_oa_event(uint64_t report_format,
                    int period_exponent,
                    double duration,
                    bool single_context,
                    int drm_fd,
                    uint32_t ctx_id,
                    void (*record_sample)(struct event *event,
                                          const struct perf_event_header *header))
{
        struct perf_event_attr attr;
        unsigned int mmap_size = 32 * page_size; /* NB: must be power of two */
        int event_fd;
        void *mmap_base;
        struct perf_event_mmap_page *mmap_page;
        struct event *event;
        struct epoll_event ev = { 0 };
        int ret;
        uint64_t period_ns;
        uint64_t n_samples;

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
                fprintf(stderr, "Error mapping circular buffer, %m\n");
                close(event_fd);
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

        fprintf(stderr, "i915 event::\n");
        fprintf(stderr, "  mmap info:\n");
        fprintf(stderr, "  base = %p\n", mmap_base);
        fprintf(stderr, "  version = %u\n", mmap_page->version);
        fprintf(stderr, "  compat version = %u\n", mmap_page->compat_version);
        fprintf(stderr, "  head = %" PRIu64 "\n", (uint64_t)mmap_page->data_head);
        fprintf(stderr, "  tail = %" PRIu64 "\n", (uint64_t)mmap_page->data_tail);

        period_ns = 80 * 2 ^ (period_exponent + 1);
        n_samples = (duration  * 1000000000.0) / period_ns;
        n_samples *= 1.1; /* a bit of leeway */

        event = malloc(sizeof (struct event));
        event->fd = event_fd;
        event->mmap_size = mmap_size;
        event->mmap_base = mmap_base;
        event->mmap_page = mmap_page;
        event->samples = malloc(n_samples * report_format_size(report_format));
        event->samples_offset = 0;
        event->record_sample = record_sample;

        events[n_events++] = event;

        ev.data.fd = event_fd;
        ev.events = EPOLLIN;
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev);

        return event;
}

static void
record_event_samples(struct event *event, bool flush)
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
        rmb();

        tail = event->mmap_page->data_tail;

        //fprintf(stderr, "Handle event mask = 0x%" PRIx64 " head=%" PRIu64 " tail=%" PRIu64 "\n", mask, head, tail);

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

                //fprintf(stderr, "header = %p tail=%" PRIu64 " size=%d\n", header, tail, header->size);

                if ((const uint8_t *)header + header->size > data + size) {
                        int before;

                        if (header->size > scratch_buffer_size) {
                                uint8_t *b = realloc(scratch_buffer, header->size);
                                if (b == NULL) {
                                        fprintf(stderr, "Failed to allocate scratch buffer for split sample\n");
                                        exit(1);
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
                                       event->record_sample(event, header);
                                       break;
                default:
                                       fprintf(stderr, "Spurious header type = %d\n", header->type);
                }

                //fprintf(stderr, "Tail += %d\n", header->size);

                tail += header->size;
        }

        event->mmap_page->data_tail = tail;
        wmb();
}

static void
record_samples_cb(bool flush)
{
        fprintf(stderr, "Recording samples...\n");

        for (int i = 0; i < n_events; i++) {
                record_event_samples(events[i], flush);
        }
}

static void
emit_process_name(int pid, const char *name)
{
        printf("},{\n");
        printf("    \"cat\": \"__metadata\",\n");
        printf("    \"name\": \"process_name\",\n");
        printf("    \"pid\": %d,\n", pid);
        printf("    \"ph\": \"M\",\n"); /* metadata event */
        printf("    \"args\": { \"name\": \"%s\"}\n", name);
}

static void
emit_process_sort_index(int pid, int sort_index)
{
        printf("},{\n");
        printf("    \"cat\": \"__metadata\",\n");
        printf("    \"name\": \"process_sort_index\",\n");
        printf("    \"pid\": %d,\n", pid);
        printf("    \"ph\": \"M\",\n"); /* metadata event */
        printf("    \"args\": { \"sort_index\": %d }\n", sort_index);
}

static void
emit_thread_name(int pid, int tid, const char *name)
{
        printf("},{\n");
        printf("    \"cat\": \"__metadata\",\n");
        printf("    \"name\": \"thread_name\",\n");
        printf("    \"pid\": %d,\n", pid);
        printf("    \"tid\": %d,\n", tid);
        printf("    \"ph\": \"M\",\n"); /* metadata event */
        printf("    \"args\": { \"name\": \"samples\"}\n");
}

static void
emit_instant_event(int pid, int tid, const char *name, uint64_t timestamp)
{
        printf("},{\n");
        printf("    \"name\": \"%s\",\n", name);
        printf("    \"pid\": %d,\n", pid);
        printf("    \"tid\": %d,\n", tid);
        printf("    \"ph\": \"I\",\n"); /* instant event */
        printf("    \"ts\": %" PRIu64 "\n", timestamp);
}

/* note although a counter can be associated with a unique thread, and
 * trace_viewer will label the track with the thread's name, counter
 * tracks are sorted separately and before threads within a process
 * so you can't interleave them with instant events for example which
 * conceptually belong to a thread. */
static void
emit_uint_counter_event(int pid, int tid,
                         const char *name, uint64_t value,
                         uint64_t timestamp)
{
        printf("},{\n");
        printf("    \"cat\": \"gpu\",\n");
        printf("    \"name\": \"%s\",\n", name);
        printf("    \"pid\": %d,\n", pid);
        printf("    \"tid\": %d,\n", tid);
        printf("    \"ph\": \"C\",\n");
        printf("    \"ts\": %" PRIu64 ",\n", timestamp);
        printf("    \"args\": { \"%s\": %" PRIu64 " }\n", name, value);
}

static void
timer_cb(void)
{
        struct event *event;
        int n_samples;
        struct oa_sample *samples;

        record_samples_cb(false /* don't force flush */);

        printf("{ \"traceEvents\": [\n");

        printf("{\n");
        printf("    \"cat\": \"__metadata\",\n");
        printf("    \"name\": \"process_name\",\n");
        printf("    \"pid\": %d,\n", (int)SAMPLES_TID);
        printf("    \"ph\": \"M\",\n"); /* metadata event */
        printf("    \"args\": { \"name\": \"intel_gpu_trace\"}\n");

        emit_process_sort_index(SAMPLES_TID, -1);

        emit_thread_name(SAMPLES_TID, SAMPLES_TID, "samples");
        emit_thread_name(SAMPLES_TID, SAMPLES_TID + 1, "samples (bad)");

        emit_process_name(CS_TID, "Command Streamer");
        emit_thread_name(CS_TID, CS_TID + 1, "context switches");

        event = i915_oa_event;
        samples = (void *)event->samples;
        n_samples = event->samples_offset / sizeof(struct oa_sample);
        for (int i = 1; i < n_samples; i++) {
                struct oa_sample *sample = samples + i;
                struct oa_sample *prev_sample = samples + i - 1;
                //uint64_t log_tsc = event_timestamp_to_tsc(event, sample->log_timestamp);
                //uint64_t ms = log_tsc / tsc_per_microsecond;
                uint64_t usec = read_counter(GPU_TIMESTAMP, sample);

                emit_instant_event(SAMPLES_TID, SAMPLES_TID, "Sample", usec);

                for (int j = 0; j < n_counters; j++) {
                        struct counter *counter = &counters[j];
                        struct value value;

                        counter->read(counter, prev_sample, sample, &value);

                        /* so that we can relate counter values to their maximum we
                         * special case the first value... */
                        if (i == 1)
                                value.value = counter->max;

#if 0
                        if (value.value > counter->max) {
                                emit_instant_event(SAMPLES_TID, SAMPLES_TID + 1,
                                                "Sample (Bad)", value.timestamp);
                                value.value = counter->max;
                        }
#endif

                        emit_uint_counter_event(counter->pid,
                                                counter->tid,
                                                counter->name,
                                                value.value,
                                                value.timestamp);
                }
        }

        printf("}");

        printf("]}\n");

        exit(0);
}

#if 0
static void
record_scalar_sample(struct event *event,
                     const struct perf_event_header *header)
{
        struct {
                struct perf_event_header header;
                uint64_t time; /* PERF_SAMPLE_TIME */
                uint64_t value; /* PERF_SAMPLE_READ */
        } *perf_sample = (void *)header;
        struct scalar_sample *sample = (void *)(event->samples + event->samples_offset);

        assert(header->type == PERF_RECORD_SAMPLE);

        if (event->samples_offset + sizeof(*sample) > SAMPLE_BUF_SIZE) {
                fprintf(stderr, "FIXME: Handle sample buffer overrun\n");
                exit(1);
        }

        //fprintf(stderr, "Recording scalar sample for event = %p: value = %" PRIu64 ", time = %" PRIu64 "\n",
        //        event, perf_sample->value, perf_sample->time);
        sample->time = perf_sample->time;
        sample->value = perf_sample->value;
        event->samples_offset += sizeof(*sample);
}
#endif

static void
record_oa_sample(struct event *event,
                 const struct perf_event_header *header)
{
        struct {
                struct perf_event_header header;
                uint64_t time; /* PERF_SAMPLE_TIME */
                uint64_t value; /* PERF_SAMPLE_READ */
                uint32_t raw_size;
                uint8_t raw_data[];
        } *perf_sample = (void *)header;
        struct oa_sample *sample = (void *)(event->samples + event->samples_offset);

        assert(header->type == PERF_RECORD_SAMPLE);
        assert((sizeof(sample->report) + 4)== perf_sample->raw_size);

        if (event->samples_offset + sizeof(*sample) > SAMPLE_BUF_SIZE) {
                fprintf(stderr, "FIXME: Handle sample buffer overrun\n");
                exit(1);
        }

        //fprintf(stderr, "Recording scalar sample for event = %p: value = %" PRIu64 ", time = %" PRIu64 "\n",
        //        event, perf_sample->value, perf_sample->time);
        sample->log_timestamp = perf_sample->time;
        sample->nominal_value = perf_sample->value;
        memcpy(&sample->report, perf_sample->raw_data, perf_sample->raw_size);

#if 0
        sample->gpu_timestamp =
                (uint64_t)sample->report.timestamp1 << 32 |
                sample->report.timestamp0;

        /* The least significant timestamp bit represents 80ns on Haswell */
        sample->gpu_timestamp *= 80;
        sample->gpu_timestamp /= 1000; /* usecs */
#endif

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

        event->samples_offset += sizeof(*sample);
}

#if 0
static void
record_trace_event(struct event *event,
                    const struct perf_event_header *header)
{
        struct sample_event {
                struct perf_event_header header;
                uint64_t time; /* PERF_SAMPLE_TIME */
        } *perf_sample = (struct sample_event *)header;
        uint64_t *sample = (void *)(event->samples + event->samples_offset);

        assert(header->type == PERF_RECORD_SAMPLE);

        if (event->samples_offset + sizeof(*sample) > SAMPLE_BUF_SIZE) {
                fprintf(stderr, "FIXME: Handle sample buffer overrun\n");
                exit(1);
        }

        //printf("Test trace @ %" PRIu64 "\n", perf_sample->time);
        *sample = perf_sample->time;
        event->samples_offset += sizeof(*sample);
}
#endif

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

static void usage(void)
{
	fprintf(stderr,
                "Usage: intel_gpu_trace_pmu [OPTIONS]\n"
                " -d,--duration <seconds>      Specify how long to trace for\n"
                " -e,--exponent <exponent>     Set the sampling period, where:"
                "\n"
                "                                period = 80 nanoseconds * 2^(exponent + 1)\n"
                "\n"
                "                              3  ~= 1   microsecond\n"
                "                              13 ~= 1   milliseconds\n"
                "                              16 ~= 10  milliseconds\n"
                "                              23 ~= 1.3 seconds\n"
                "\n"
                "                              Use a period < ~ 50 milliseconds to ensure\n"
                "                              counter wrapping won't cause problems\n"
                "\n"
                " -h,--help                    Display this help and exit\n");
	exit(1);
}

int
main(int argc, char **argv)
{
        sigset_t mask;
        struct epoll_event ev = { 0 };
        int timer_fd;
        struct itimerspec period;
        struct stat;
        int i;
        int drm_fd;
        int opt;
        double duration = 1;
        int period_exponent;
	drm_intel_context *context;
	drm_intel_bufmgr *bufmgr;
#if 0
	igt_render_copyfunc_t render_copy = NULL;
	struct intel_batchbuffer *batch = NULL;
	struct igt_buf src, dst;
#endif
        static const struct option long_options[] = {
                { "duration", required_argument, NULL, 'd' },
                { "exponent", required_argument, NULL, 'e' },
                { "help",     no_argument,       NULL, 'h' },
                { },
        };

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

        while((opt = getopt_long(argc, argv, "d:e:h", long_options, NULL)) != -1) {
		switch (opt) {
		case 'd':
                        duration = fabs(strtod(optarg, NULL));
                        continue;
                case 'e':
                        period_exponent = strtoul(optarg, NULL, 10);
                        if (period_exponent > 25) {
                                fprintf(stderr, "Capping exponent to 25 (~ 5 seconds)\n");
                                period_exponent = 25;
                        }
                        continue;
		case 'h':
		case '?': /* error */
                case ':': /* missing arg */
                        usage();
                }
        }

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

        scratch_buffer_size = SAMPLE_BUF_SIZE;
        scratch_buffer = malloc(scratch_buffer_size);

        calibrate_tsc_to_hz();

        epoll_fd = epoll_create1(EPOLL_CLOEXEC);

        sigemptyset(&mask);
        sigaddset(&mask, SIGINT);
        sigprocmask(SIG_BLOCK, &mask, NULL);

        sigint_fd = signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);

        ev.data.fd = sigint_fd;
        ev.events = EPOLLIN;
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, sigint_fd, &ev);

        timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC | TFD_NONBLOCK);
        period.it_value.tv_sec = duration;
        period.it_value.tv_nsec = (duration - (int)duration) * 1000000000.0;
        period.it_interval.tv_sec = period.it_value.tv_sec;
        period.it_interval.tv_nsec = period.it_value.tv_nsec;
        timerfd_settime(timer_fd, 0, &period, NULL);

        ev.data.fd = timer_fd;
        ev.events = EPOLLIN;
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_fd, &ev);

#if 1
        i915_oa_event = open_i915_oa_event(I915_PERF_OA_FORMAT_A45_B8_C8_HSW,
                                           period_exponent,
                                           duration,
                                           false, /* profile cross-context */
                                           -1, /* drm fd ignored */
                                           0, /* ctx-id ignored */
                                           record_oa_sample);
#else
        i915_oa_event = open_i915_oa_event(I915_PERF_OA_FORMAT_A45_B8_C8_HSW,
                                           period_exponent,
                                           true, /* profile single context */
                                           drm_fd,
                                           context->ctx_id,
                                           record_oa_sample);
#endif

        if (!i915_oa_event) {
                fprintf(stderr, "Failed to open i915 OA event\n");
                return EXIT_FAILURE;
        }

        for (;;) {
                struct epoll_event poll_events[256];
                int n = epoll_wait(epoll_fd, poll_events, 256, -1);

#if 0
                for (i = 0; i < 10000; i++)
                        render_copy(batch, context,
                                    &src, 0, 0, SCRATCH_BO_WIDTH, SCRATCH_BO_HEIGHT,
                                    &dst, SCRATCH_BO_WIDTH / 2, SCRATCH_BO_HEIGHT / 2);
#endif

                for (i = 0; i < n; i++) {
                        if (poll_events[i].data.fd == sigint_fd) {
                                record_samples_cb(true); /* force flush */
                                goto quit;
                        }
                        else if (poll_events[i].data.fd == timer_fd) {
                                uint64_t ticks;
                                read(timer_fd, &ticks, 8);
                                timer_cb();
                        } else
                                record_samples_cb(false);
                }
        }

quit:
        fprintf(stderr, "Quit\n");

        return 0;
}
