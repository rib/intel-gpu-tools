/*
 * Copyright © 2007 Intel Corporation
 * Copyright © 2011 Intel Corporation
 * Copyright © 2014 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *    Eric Anholt <eric@anholt.net>
 *    Eugeni Dodonov <eugeni.dodonov@intel.com>
 *    Robert Bragg <robert.bragg@intel.com>
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <err.h>
#include <time.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <string.h>
#ifdef HAVE_TERMIOS_H
#include <termios.h>
#endif
#include "intel_io.h"
#include "instdone.h"
#include "intel_reg.h"
#include "intel_chipset.h"

//#define DURATION 1000000 /* microseconds */
//#define DURATION	    125000 /* microseconds */
//#define DURATION	    62500 /* microseconds */
#define DURATION	    31250 /* microseconds */

#define SAMPLES_PER_FRAME	    4000
#define SAMPLES_PER_SEC             (SAMPLES_PER_FRAME * 60)
#define SAMPLES_TO_PERCENT_RATIO    (SAMPLES_PER_SEC / 100)

#define MAX_NUM_TOP_BITS            100

#define HAS_STATS_REGS(devid)	    IS_965(devid)

#define SAMPLES_TID         1234
#define CS_TID              (SAMPLES_TID + 100)
#define STATS_TID_BASE      (CS_TID + 100)
#define INSTDONE_TID_BASE   (STATS_TID_BASE + STATS_COUNT)


struct top_bit {
	struct instdone_bit *bit;
	int count;
} top_bits[MAX_NUM_TOP_BITS];
struct top_bit *top_bits_sorted[MAX_NUM_TOP_BITS];

enum stats_counts {
	IA_VERTICES,
	IA_PRIMITIVES,
	VS_INVOCATION,
	GS_INVOCATION,
	GS_PRIMITIVES,
	CL_INVOCATION,
	CL_PRIMITIVES,
	PS_INVOCATION,
	PS_DEPTH,
	STATS_COUNT
};

const uint32_t stats_regs[STATS_COUNT] = {
	IA_VERTICES_COUNT_QW,
	IA_PRIMITIVES_COUNT_QW,
	VS_INVOCATION_COUNT_QW,
	GS_INVOCATION_COUNT_QW,
	GS_PRIMITIVES_COUNT_QW,
	CL_INVOCATION_COUNT_QW,
	CL_PRIMITIVES_COUNT_QW,
	PS_INVOCATION_COUNT_QW,
	PS_DEPTH_COUNT_QW,
};

const char *stats_reg_names[STATS_COUNT] = {
	"vert fetch",
	"prim fetch",
	"VS invocations",
	"GS invocations",
	"GS prims",
	"CL invocations",
	"CL prims",
	"PS invocations",
	"PS depth pass",
};

struct sample {
	uint64_t timestamp;
        uint32_t ccid_start;
        uint32_t ccid_end;
	uint32_t head, tail;
	uint32_t instdone;
	uint32_t instdone1;
	uint64_t stats[STATS_COUNT];
	unsigned bad:1;
};

struct ring {
	const char *name;
	uint32_t mmio;
	uint32_t head, tail, size;
	uint64_t full;
	int idle;
	unsigned enabled:1;
};

static struct ring render_ring = {
        .name = "render",
        .mmio = 0x2030,
}, bsd_ring = {
        .name = "bitstream",
        .mmio = 0x4030,
}, bsd6_ring = {
        .name = "bitstream",
        .mmio = 0x12030,
}, blt_ring = {
        .name = "blitter",
        .mmio = 0x22030,
};
    
static unsigned long gt_max_freq_mhz = 0;

static uint32_t ring_read(struct ring *ring, uint32_t reg)
{
	return INREG(ring->mmio + reg);
}

static void ring_init(struct ring *ring)
{
	ring->size = (((ring_read(ring, RING_CTL) & RING_NR_PAGES) >> 12) + 1) * 4096;
}

static void ring_reset(struct ring *ring)
{
	ring->idle = ring->full = 0;
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

static void
emit_process_name (int pid, const char *name)
{
        printf ("},{\n");
        printf ("    \"cat\": \"__metadata\",\n");
        printf ("    \"name\": \"process_name\",\n");
        printf ("    \"pid\": %d,\n", (int)SAMPLES_TID);
        printf ("    \"ph\": \"M\",\n"); /* metadata event */
        printf ("    \"args\": { \"name\": \"%s\"}\n", name);
}

static void
emit_process_sort_index (int pid, int sort_index)
{
        printf ("},{\n");
        printf ("    \"cat\": \"__metadata\",\n");
        printf ("    \"name\": \"process_sort_index\",\n");
        printf ("    \"pid\": %d,\n", (int)SAMPLES_TID);
        printf ("    \"ph\": \"M\",\n"); /* metadata event */
        printf ("    \"args\": { \"sort_index\": %d }\n", sort_index);
}

static void
emit_thread_name (int pid, int tid, const char *name)
{
        printf ("},{\n");
        printf ("    \"cat\": \"__metadata\",\n");
        printf ("    \"name\": \"thread_name\",\n");
        printf ("    \"pid\": %d,\n", pid);
        printf ("    \"tid\": %d,\n", tid);
        printf ("    \"ph\": \"M\",\n"); /* metadata event */
        printf ("    \"args\": { \"name\": \"samples\"}\n");
}

static void
emit_instant_event (int pid, int tid, const char *name, uint64_t timestamp)
{
        printf ("},{\n");
        printf ("    \"name\": \"Sample (Bad)\",\n");
        printf ("    \"pid\": %d,\n", pid);
        printf ("    \"tid\": %d,\n", tid);
        printf ("    \"ph\": \"I\",\n"); /* instant event */
        printf ("    \"ts\": %" PRIu64 "\n", timestamp);
}

/* note although a counter can be associated with a unique thread, and
 * trace_viewer will label the track with the thread's name, counter
 * tracks are sorted separately and before threads within a process
 * so you can't interleave them with instant events for example which
 * conceptually belong to a thread. */
static void
emit_uint_counter_event (int pid, int tid,
                         const char *name, uint64_t value,
                         uint64_t timestamp)
{
        printf ("},{\n");
        printf ("    \"cat\": \"gpu\",\n");
        printf ("    \"name\": \"%s\",\n", name);
        printf ("    \"pid\": %d,\n", pid);
        printf ("    \"tid\": %d,\n", tid);
        printf ("    \"ph\": \"C\",\n");
        printf ("    \"ts\": %" PRIu64 ",\n", timestamp);
        printf ("    \"args\": { \"%s\": %" PRIu64 " }\n", name, value);
}

static void
emit_context_changes (struct sample *samples, int n_samples)
{
        int i;

        /* we're walking in pairs, so don't overrun... */
        n_samples--;

        for (i = 0; i < n_samples; i++) {
                struct sample *s0 = &samples[i];
                struct sample *s1 = &samples[i + 1];
                uint64_t ms;

                if (s0->ccid_start != s0->ccid_end)
                        ms = s0->timestamp;
                else if (s0->ccid_end != s1->ccid_start)
                        ms = s0->timestamp + (s1->timestamp - s0->timestamp) / 2;
                else
                        continue;

                emit_instant_event(CS_TID, CS_TID + 1, "Context Switch", ms);
        }
}

static void
emit_stat_reg_metrics (int reg, struct sample *samples, int n_samples)
{
        struct sample *first_s;
        int i;
        int j;

        /* XXX: consider that in some pathological cases we may be
         * switching contexts at too high a frequency for us to
         * aggregate the data. */
#define AGGREGATE_COUNT 10
//#define AGGREGATE_COUNT 2

        /* we're walking in pairs, so don't overrun... */
        n_samples--;

        for (i = j = 0; i < n_samples; i++) {
                struct sample *s0 = &samples[i];
                struct sample *s1 = &samples[i + 1];
                uint64_t dt;
                uint64_t dv;
                double grad;
                double grad_sum = 0;

                if (s0->bad || s1->bad)
                        continue;

                if (s0->ccid_end != s0->ccid_start) {
                        j = 0;
                        continue;
                }

                dt = s1->timestamp - s0->timestamp;
                dv = s1->stats[reg] - s0->stats[reg];

                grad = dv / dt;
                grad_sum += grad;

                j++;

                if (j == 1)
                        first_s = s0;
                else if (j == AGGREGATE_COUNT) {
                        uint64_t av;
                        uint64_t ms;

                        /* TODO: normalize each stat so that its
                         * peak rate of increase is 1 per GPU clock
                         * cycle.  So for example if we know we can
                         * theoretically handle up to 2 pixels per EU
                         * per clock and we have 4 EUs then we would
                         * scale the PS_INVOCATION_COUNT by 1/8.
                         *
                         * TODO: Use the known max GPU frequency and
                         * the peak rate to emit samples as a
                         * fraction/percentage of the peak throughput.
                         *
                         * I think this will give us a quite intuitive
                         * view of the data where 100% represents peak
                         * throughput with the GPU running at full
                         * tilt. Notably this will expose the effects
                         * of frequency scaling on performance which I
                         * think is important.
                         */
                        av = grad_sum / AGGREGATE_COUNT;

                        ms = first_s->timestamp + (s1->timestamp - first_s->timestamp) / 2;

                        printf ("},{\n");
                        printf ("    \"name\": \"%s (derivative)\",\n", stats_reg_names[reg]);
                        printf ("    \"pid\": %d,\n", reg + STATS_TID_BASE);
                        printf ("    \"tid\": %d,\n", reg + STATS_TID_BASE);
                        printf ("    \"ph\": \"C\",\n");
                        printf ("    \"ts\": %" PRIu64 ",\n", ms);
                        printf ("    \"args\": { \"value\": %" PRIu64 "}\n", av);

                        printf ("},{\n");
                        printf ("    \"name\": \"%s (derivative sample)\",\n", stats_reg_names[reg]);
                        printf ("    \"pid\": %d,\n", reg + STATS_TID_BASE);
                        printf ("    \"tid\": %d,\n", reg + STATS_TID_BASE);
                        printf ("    \"ph\": \"I\",\n");
                        printf ("    \"ts\": %" PRIu64 "\n", ms);

                        grad_sum = 0;
                        j = 0;
                }
        }
}

int main(int argc, char **argv)
{
	uint32_t devid;
	struct pci_device *pci_dev;
	int i;
	int samples_per_sec = SAMPLES_PER_SEC;
	struct sample *samples;
	int n_samples;
	uint64_t period;
	uint64_t start_ts, ts1;
	FILE *file;
	char *line = NULL;
	size_t line_size;

	file = fopen ("/sys/class/drm/card0/gt_max_freq_mhz", "r");
	if (getline(&line, &line_size, file) < 0) {
		fprintf(stderr, "Failed to read max GPU frequency "
			"required to normalize some metricss");
		exit(1);
	}
        gt_max_freq_mhz = strtoul (line, NULL, 10);
	free (line);

	calibrate_tsc_to_hz();

	pci_dev = intel_get_pci_device();
	devid = pci_dev->device_id;
	intel_mmio_use_pci_bar(pci_dev);
	init_instdone_definitions(devid);

	/* Grab access to the registers */
	intel_register_access_init(pci_dev, 0);

	ring_init(&render_ring);
	if (IS_GEN4(devid) || IS_GEN5(devid))
		ring_init(&bsd_ring);
	if (IS_GEN6(devid) || IS_GEN7(devid)) {
		ring_init(&bsd6_ring);
		ring_init(&blt_ring);
	}

	ring_reset(&render_ring);
	ring_reset(&bsd_ring);
	ring_reset(&bsd6_ring);
	ring_reset(&blt_ring);

	n_samples = (DURATION * samples_per_sec) / 1000000;
        //n_samples = 100;
	samples = malloc (n_samples * sizeof (struct sample));
	period = 1000000 / samples_per_sec;
	start_ts = ts1 = __rdtsc () / tsc_per_microsecond;

	for (i = 0; i < n_samples; i++) {
		struct sample *s = &samples[i];
		uint64_t ts0 = ts1;
		int j;

		if (IS_965(devid)) {
			s->instdone = INREG(INSTDONE_I965);
			s->instdone1 = INREG(INSTDONE_1);
		} else
			s->instdone = INREG(INSTDONE);

                s->ccid_start = ring_read(&render_ring, RING_CCID) & CCID_ADDR_MASK;

		s->head = ring_read(&render_ring, RING_HEAD) & HEAD_ADDR;
		s->tail = ring_read(&render_ring, RING_TAIL) & TAIL_ADDR;

		if (HAS_STATS_REGS(devid)) {
			for (j = 0; j < STATS_COUNT; j++) {
				uint32_t stats_high, stats_low, stats_high_2;

				do {
					stats_high = INREG(stats_regs[j] + 4);
					stats_low = INREG(stats_regs[j]);
					stats_high_2 = INREG(stats_regs[j] + 4);
				} while (stats_high != stats_high_2);

				s->stats[j] = (uint64_t)stats_high << 32 |
					stats_low;
			}
		}

                s->ccid_end = ring_read(&render_ring, RING_CCID) & CCID_ADDR_MASK;
		s->timestamp = ts0;

		do {
			ts1 = __rdtsc() / tsc_per_microsecond;
		} while((ts1 - ts0) < period);
	}

	intel_register_access_fini();


	for (i = 0; i < n_samples; i++) {
		struct sample *s = &samples[i];

                /* We sometimes spurious, out of range pointers which
                 * we want to ignore... */
		if (s->head > render_ring.size ||
		    s->tail > render_ring.size) {
                        s->bad = 1;
			continue;
		}

                /* Some of the stats are per-context so we have bad
                 * data if the context changed while sampling... */
                if (s->ccid_start != s->ccid_end) {
                        s->bad = 1;
                }
        }

#if 0
	for (i = 0; i < n_samples; i++) {
		struct sample *s = &samples[i];
                printf ("ccid start=%" PRIu32 " end=%" PRIu32 " bad=%d\n",
                                s->ccid_start, s->ccid_end, s->bad);
        }
        exit (0);
#endif

	printf ("{ \"traceEvents\": [\n");


        printf ("{\n");
        printf ("    \"cat\": \"__metadata\",\n");
        printf ("    \"name\": \"process_name\",\n");
        printf ("    \"pid\": %d,\n", (int)SAMPLES_TID);
        printf ("    \"ph\": \"M\",\n"); /* metadata event */
        printf ("    \"args\": { \"name\": \"intel_gpu_trace\"}\n");

        emit_process_sort_index (SAMPLES_TID, -1);

        emit_thread_name (SAMPLES_TID, SAMPLES_TID, "samples");
        emit_thread_name (SAMPLES_TID, SAMPLES_TID + 1, "samples (bad)");

        emit_process_name (CS_TID, "Command Streamer");
        emit_thread_name (CS_TID, CS_TID + 1, "context switches");

        for (i = 0; i < STATS_COUNT; i++) {
                emit_process_name (i + STATS_TID_BASE, stats_reg_names[i]);
                emit_process_sort_index (i + STATS_TID_BASE, i + STATS_TID_BASE);
        }

        for (i = 0; i < num_instdone_bits; i++)
                emit_process_name (i + INSTDONE_TID_BASE, instdone_bits[i].name);

        emit_context_changes (samples, n_samples);

	for (i = 0; i < n_samples; i++) {
		struct sample *s = &samples[i];
		uint64_t ms = s->timestamp;
		int32_t fill;
		int j;

                if (s->bad) {
                        emit_instant_event (SAMPLES_TID, SAMPLES_TID + 1,
                                            "Sample (Bad)", ms);
                        continue;
                }

                fill = s->tail - s->head;
		if (fill > render_ring.size)
			fill += render_ring.size;

                emit_instant_event (SAMPLES_TID, SAMPLES_TID, "Sample", ms);

                emit_uint_counter_event (CS_TID, CS_TID, "ring space", fill, ms);
                emit_uint_counter_event (CS_TID, CS_TID, "ring active",
                                         s->head != s->tail, ms);

                printf ("},{\n");

                printf ("    \"cat\": \"gpu\",\n");
                if (s->head != s->tail) {
                        printf ("    \"name\": \"CS active (context = %" PRIu32 ")\",\n", s->ccid_start);
                } else {
                        printf ("    \"name\": \"CS idle\",\n");
                }
                printf ("    \"pid\": %d,\n", (int)CS_TID);
                printf ("    \"tid\": %d,\n", (int)CS_TID);
                printf ("    \"ph\": \"P\",\n");
                printf ("    \"ts\": %" PRIu64 "\n", ms);

                emit_uint_counter_event (CS_TID, CS_TID, "ring head (consumer)",
                                         s->head, ms);
                emit_uint_counter_event (CS_TID, CS_TID, "ring tail (producer)",
                                         s->tail, ms);

		for (j = 0; j < num_instdone_bits; j++) {
                        bool active;

                        printf ("},{\n");

                        if (instdone_bits[j].reg == INSTDONE_1) {
                                active = !(s->instdone1 & instdone_bits[j].bit);
                        } else {
                                active = !(s->instdone & instdone_bits[j].bit);
                        }

                        if (active) {
                                printf ("    \"name\": \"%s active\",\n",
                                        instdone_bits[j].name);
                        } else {
                                printf ("    \"name\": \"%s idle\",\n",
                                        instdone_bits[j].name);
                        }

                        printf ("    \"pid\": %d,\n", j + INSTDONE_TID_BASE);
                        printf ("    \"tid\": %d,\n", j + INSTDONE_TID_BASE);
                        printf ("    \"ph\": \"P\",\n");
                        printf ("    \"ts\": %" PRIu64 "\n", ms);

                        /* Counters are a bit easier to visualize in
                         * trace_viewer but the samples above are
                         * give us accumulated percentages which are
                         * useful too */
                        if (active) {
                                emit_uint_counter_event (j + INSTDONE_TID_BASE,
                                                         j + INSTDONE_TID_BASE,
                                                         instdone_bits[j].name,
                                                         1, ms);
                        }
		}

		for (j = 0; j < STATS_COUNT; j++) {
                        emit_uint_counter_event (j + STATS_TID_BASE,
                                                 j + STATS_TID_BASE,
                                                 stats_reg_names[j],
                                                 s->stats[j], ms);
		}
	}

	for (i = 0; i < STATS_COUNT; i++) {
	        emit_stat_reg_metrics (i, samples, n_samples);
	}

	printf ("}");

	printf ("]}\n");

	return 0;
}
