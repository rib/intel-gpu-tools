/*
 * Copyright © 2007 Intel Corporation
 * Copyright © 2011 Intel Corporation
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

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <err.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifdef HAVE_TERMIOS_H
#include <termios.h>
#endif
#include "intel_io.h"
#include "instdone.h"
#include "intel_reg.h"
#include "intel_chipset.h"

#define  FORCEWAKE	    0xA18C
#define  FORCEWAKE_ACK	    0x130090

/* Aim for ~2000 samples per frame @ 60fps... */
#define SAMPLES_PER_SEC             (60 * 2000)
#define SAMPLES_TO_PERCENT_RATIO    (SAMPLES_PER_SEC / 100)

#define MAX_NUM_TOP_BITS            100

#define HAS_STATS_REGS(devid)		IS_965(devid)

struct top_bit {
	struct instdone_bit *bit;
	int count;
} top_bits[MAX_NUM_TOP_BITS];
struct top_bit *top_bits_sorted[MAX_NUM_TOP_BITS];

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

struct pipeline_stat {
	uint64_t start;
	uint64_t end;
	unsigned warped:1;
};

#define MAX_CONTEXTS 10
struct context {
	uint32_t id;
	struct pipeline_stat stats[STATS_COUNT];
	int n_samples;
} contexts[MAX_CONTEXTS];
struct context *contexts_sorted[MAX_CONTEXTS];

enum rings {
        RING_RENDER,
        RING_BSD,
        RING_BSD6,
        RING_BLIT,
        MAX_RINGS
};

static struct ring {
	const char *name;
	uint32_t mmio;
	int size;

	/* used for analytics... */
	uint64_t full;
	uint64_t idle;
	int n_samples;
} rings[MAX_RINGS] = {
	{ .name = "render",    .mmio = 0x2030 },
	{ .name = "bitstream", .mmio = 0x4030 },
	{ .name = "bitstream", .mmio = 0x12030 },
	{ .name = "blitter",   .mmio = 0x22030 }
};

struct ring_sample {
        uint32_t ccid_start;
        uint32_t ccid_end;
        uint32_t head, tail;
};

struct sample {
	uint64_t timestamp;
	struct ring_sample ring_samples[MAX_RINGS];
	uint32_t instdone;
	uint32_t instdone1;
	uint64_t stats[STATS_COUNT];
};

static unsigned long
gettime(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return (t.tv_usec + (t.tv_sec * 1000000));
}

static int
read_file_int(const char *file)
{
	char buf[32];
	int fd, n;

	fd = open(file, 0);
	if (fd < 0)
	    return -1;
	n = read(fd, buf, sizeof (buf) - 1);
	close(fd);
	if (n < 0)
	    return -1;

	buf[n] = '\0';
	return strtol(buf, 0, 0);
}

static int
top_bits_sort(const void *a, const void *b)
{
	struct top_bit * const *bit_a = a;
	struct top_bit * const *bit_b = b;
	int a_count = (*bit_a)->count;
	int b_count = (*bit_b)->count;

	if (a_count < b_count)
		return 1;
	else if (a_count == b_count)
		return 0;
	else
		return -1;
}

static int
contexts_sort(const void *a, const void *b)
{
	struct context * const * context_a = a;
	struct context * const * context_b = b;
	int a_samples = (*context_a)->n_samples;
	int b_samples = (*context_b)->n_samples;

	if (a_samples < b_samples)
		return 1;
	else if (a_samples == b_samples)
		return 0;
	else
		return -1;
}

static void
print_clock(const char *name, int clock) {
	if (clock == -1)
		printf("%s clock: unknown", name);
	else
		printf("%s clock: %d Mhz", name, clock);
}

static int
print_clock_info(struct pci_device *pci_dev)
{
	uint32_t devid = pci_dev->device_id;
	uint16_t gcfgc;

	if (IS_GM45(devid)) {
		int core_clock = -1;

		pci_device_cfg_read_u16(pci_dev, &gcfgc, I915_GCFGC);

		switch (gcfgc & 0xf) {
		case 8:
			core_clock = 266;
			break;
		case 9:
			core_clock = 320;
			break;
		case 11:
			core_clock = 400;
			break;
		case 13:
			core_clock = 533;
			break;
		}
		print_clock("core", core_clock);
	} else if (IS_965(devid) && IS_MOBILE(devid)) {
		int render_clock = -1, sampler_clock = -1;

		pci_device_cfg_read_u16(pci_dev, &gcfgc, I915_GCFGC);

		switch (gcfgc & 0xf) {
		case 2:
			render_clock = 250; sampler_clock = 267;
			break;
		case 3:
			render_clock = 320; sampler_clock = 333;
			break;
		case 4:
			render_clock = 400; sampler_clock = 444;
			break;
		case 5:
			render_clock = 500; sampler_clock = 533;
			break;
		}

		print_clock("render", render_clock);
		printf("  ");
		print_clock("sampler", sampler_clock);
	} else if (IS_945(devid) && IS_MOBILE(devid)) {
		int render_clock = -1, display_clock = -1;

		pci_device_cfg_read_u16(pci_dev, &gcfgc, I915_GCFGC);

		switch (gcfgc & 0x7) {
		case 0:
			render_clock = 166;
			break;
		case 1:
			render_clock = 200;
			break;
		case 3:
			render_clock = 250;
			break;
		case 5:
			render_clock = 400;
			break;
		}

		switch (gcfgc & 0x70) {
		case 0:
			display_clock = 200;
			break;
		case 4:
			display_clock = 320;
			break;
		}
		if (gcfgc & (1 << 7))
		    display_clock = 133;

		print_clock("render", render_clock);
		printf("  ");
		print_clock("display", display_clock);
	} else if (IS_915(devid) && IS_MOBILE(devid)) {
		int render_clock = -1, display_clock = -1;

		pci_device_cfg_read_u16(pci_dev, &gcfgc, I915_GCFGC);

		switch (gcfgc & 0x7) {
		case 0:
			render_clock = 160;
			break;
		case 1:
			render_clock = 190;
			break;
		case 4:
			render_clock = 333;
			break;
		}
		if (gcfgc & (1 << 13))
		    render_clock = 133;

		switch (gcfgc & 0x70) {
		case 0:
			display_clock = 190;
			break;
		case 4:
			display_clock = 333;
			break;
		}
		if (gcfgc & (1 << 7))
		    display_clock = 133;

		print_clock("render", render_clock);
		printf("  ");
		print_clock("display", display_clock);
	} else {
	    int max_render_clock;
	    int cur_render_clock;

	    max_render_clock = read_file_int("/sys/class/drm/card0/gt_max_freq_mhz");
	    cur_render_clock = read_file_int("/sys/class/drm/card0/gt_cur_freq_mhz");

	    print_clock("max render", max_render_clock);
	    printf("  ");
	    print_clock("current render", cur_render_clock);
	}


	printf("\n");
	return -1;
}

#define STATS_LEN (20)
#define PERCENTAGE_BAR_END	(79 - STATS_LEN)

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

static uint32_t ring_read(struct ring *ring, uint32_t reg)
{
	return INREG(ring->mmio + reg);
}

static void ring_init(struct ring *ring)
{
	ring->size = (((ring_read(ring, RING_CTL) & RING_NR_PAGES) >> 12) + 1) * 4096;
}

static void ring_print_header(FILE *out, struct ring *ring)
{
	fprintf(out, " %9s%% %6s", ring->name, "ops");
}

static void ring_print(struct ring *ring)
{
	int percent_busy, len;

	if (!ring->size)
		return;

	percent_busy = 100 - 100 * ring->idle / ring->n_samples;

	len = printf("%25s busy: %3d%%: ", ring->name, percent_busy);
	print_percentage_bar (percent_busy, len);
	printf("%24s space: %d/%d\n",
	       ring->name,
	       (int)(ring->full / ring->n_samples),
	       ring->size);
}

static void ring_log(struct ring *ring, FILE *output)
{
	if (ring->size)
		fprintf(output, " %10d %6d",
			(int)(100 - 100 * ring->idle / ring->n_samples),
			(int)(ring->full / ring->n_samples));
	else
		fprintf(output, " %10d %6d", -1, -1);
}

static void
usage(const char *appname)
{
	printf("intel_gpu_top - Display a top-like summary of Intel GPU usage\n"
			"\n"
			"usage: %s [parameters]\n"
			"\n"
			"The following parameters apply:\n"
			"[-s <samples>]       samples per seconds (default %d)\n"
			"[-e <command>]       command to profile\n"
			"[-o <file>]          output statistics to file. If file is '-',"
			"                     run in batch mode and output statistics to stdio only \n"
			"[-h]                 show this help screen\n"
			"\n",
			appname,
			SAMPLES_PER_SEC
		  );
	return;
}

static int analyse_samples(uint32_t devid, struct sample *samples, int n_samples)
{
        int n_contexts = 0;
        int i;

        for (i = 0; i < MAX_CONTEXTS; i++)
                contexts[i].n_samples = 0;

        for (i = 0; i < n_samples; i++) {
                struct sample *sample = samples + i;
                uint32_t ccid = sample->ring_samples[RING_RENDER].ccid_start;
                struct context *context = NULL;
                int bad_render_ring_sample = 0;

                for (int j = 0; j < num_instdone_bits; j++) {
                        struct top_bit *top_bit = top_bits + j;
                        uint32_t reg_val;

                        if (top_bit->bit->reg == INSTDONE_1)
                                reg_val = sample->instdone1;
                        else
                                reg_val = sample->instdone;

                        if ((reg_val & top_bit->bit->bit) == 0)
                                top_bit->count++;
                }

                for (int j = 0; j < MAX_RINGS; j++) {
                        struct ring_sample *rs = sample->ring_samples + j;

                        if (!rings[j].size)
                                continue;

                        /* We sometimes read spurious, out of range
                         * pointers which we want to ignore... */
                        if (rs->head < rings[j].size &&
                            rs->tail < rings[j].size)
                        {
                                int32_t full = rs->tail - rs->head;

                                full = rs->tail - rs->head;
                                if (full < 0)
                                        full += rings[j].size;
                                rings[j].full += full;

                                if (!full)
                                        rings[j].idle++;

                                rings[j].n_samples++;
                        } else if (j == RING_RENDER)
                                bad_render_ring_sample = 1;
                }

                /* Some of the stats are per render context so we
                 * have bad data if the context changed while
                 * sampling... */
                if (bad_render_ring_sample ||
                    ccid != sample->ring_samples[RING_RENDER].ccid_end)
                        continue;

                for (int j = 0; j < n_contexts; j++) {
                        context = contexts + j;
                        if (context->id == ccid)
                                break;
                }
                if (n_contexts && context->id == ccid) {
                        context->n_samples++;

                        if (!HAS_STATS_REGS(devid))
                                continue;

                        for (int j = 0; j < STATS_COUNT; j++) {
                                if (sample->stats[j] >= context->stats[j].end)
                                        context->stats[j].end = sample->stats[j];
                                else
                                        context->stats[j].warped = 1;
                        }
                } else {
                        if (n_contexts == MAX_CONTEXTS)
                                continue;

                        context = &contexts[n_contexts++];
                        context->id = ccid;
                        context->n_samples = 1;

                        if (!HAS_STATS_REGS(devid))
                                continue;

                        for (int j = 0; j < STATS_COUNT; j++) {
                                context->stats[j].start = sample->stats[j];
                                context->stats[j].end = sample->stats[j];
                                context->stats[j].warped = 0;
                        }
                }
        }

        return n_contexts;
}

int main(int argc, char **argv)
{
	uint32_t devid;
	struct pci_device *pci_dev;
	int i, ch;
	int samples_per_sec = SAMPLES_PER_SEC;
	FILE *output = NULL;
	double elapsed_time=0;
	int print_headers=1;
	pid_t child_pid=-1;
	int child_stat;
	char *cmd=NULL;
	int interactive=1;
	struct sample *samples;

	/* Parse options? */
	while ((ch = getopt(argc, argv, "s:o:e:h")) != -1) {
		switch (ch) {
		case 'e': cmd = strdup(optarg);
			break;
		case 's': samples_per_sec = atoi(optarg);
			if (samples_per_sec < 100) {
				fprintf(stderr, "Error: samples per second must be >= 100\n");
				exit(1);
			}
			break;
		case 'o':
			if (!strcmp(optarg, "-")) {
				/* Running in non-interactive mode */
				interactive = 0;
				output = stdout;
			}
			else
				output = fopen(optarg, "w");
			if (!output)
			{
				perror("fopen");
				exit(1);
			}
			break;
		case 'h':
			usage(argv[0]);
			exit(0);
			break;
		default:
			fprintf(stderr, "Invalid flag %c!\n", (char)optopt);
			usage(argv[0]);
			exit(1);
			break;
		}
	}

	samples = malloc(sizeof(struct sample) * samples_per_sec);

	pci_dev = intel_get_pci_device();
	devid = pci_dev->device_id;
	intel_mmio_use_pci_bar(pci_dev);
	init_instdone_definitions(devid);

	/* Do we have a command to run? */
	if (cmd != NULL) {
		if (output) {
			fprintf(output, "# Profiling: %s\n", cmd);
			fflush(output);
		}
		child_pid = fork();
		if (child_pid < 0) {
			perror("fork");
			exit(1);
		}
		else if (child_pid == 0) {
			int res;
			res = system(cmd);
			if (res < 0)
				perror("running command");
			if (output) {
				fflush(output);
				fprintf(output, "# %s exited with status %d\n", cmd, res);
				fflush(output);
			}
			free(cmd);
			exit(0);
		} else {
			free(cmd);
		}
	}

	for (i = 0; i < num_instdone_bits; i++) {
		top_bits[i].bit = &instdone_bits[i];
		top_bits[i].count = 0;
		top_bits_sorted[i] = &top_bits[i];
	}

	for (i = 0; i < MAX_CONTEXTS; i++)
		contexts_sorted[i] = &contexts[i];

	/* Grab access to the registers */
	intel_register_access_init(pci_dev, 0);

	ring_init(&rings[RING_RENDER]);
	if (IS_GEN4(devid) || IS_GEN5(devid))
		ring_init(&rings[RING_BSD]);
	if (IS_GEN6(devid) || IS_GEN7(devid)) {
		ring_init(&rings[RING_BSD6]);
		ring_init(&rings[RING_BLIT]);
	}

	for (;;) {
		unsigned long long t1, ti, tf, t2;
		unsigned long long def_sleep = 1000000 / samples_per_sec;
		unsigned long long last_samples_per_sec = samples_per_sec;
		unsigned short int max_lines;
		struct winsize ws;
		char clear_screen[] = {0x1b, '[', 'H',
				       0x1b, '[', 'J',
				       0x0};
		int len;
		int n_contexts;

		t1 = gettime();

		for (i = 0; i < samples_per_sec; i++) {
			struct sample *sample = samples + i;
			long long interval;
			ti = gettime();

			sample->timestamp = t1;

			if (IS_965(devid)) {
				sample->instdone = INREG(INSTDONE_I965);
				sample->instdone1 = INREG(INSTDONE_1);
			} else
				sample->instdone = INREG(INSTDONE);

			for (int j = 0; j < MAX_RINGS; j++) {
				struct ring_sample *rs;

                                if (!rings[j].size)
                                        continue;

                                rs = sample->ring_samples + j;
				rs->ccid_start =
				    ring_read(rings + j, RING_CCID) & CCID_ADDR_MASK;
				rs->head =
				    ring_read(rings + j, RING_HEAD) & HEAD_ADDR;
				rs->tail =
				    ring_read(rings + j, RING_TAIL) & TAIL_ADDR;
			}

			if (HAS_STATS_REGS(devid)) {
				for (int j = 0; j < STATS_COUNT; j++) {
					uint32_t stats_high, stats_low, stats_high_2;

					do {
					    stats_high = INREG(stats_regs[j] + 4);
					    stats_low = INREG(stats_regs[j]);
					    stats_high_2 = INREG(stats_regs[j] + 4);
					} while (stats_high != stats_high_2);

					sample->stats[j] = (uint64_t)stats_high << 32 |
					    stats_low;
				}
			}

			for (int j = 0; j < MAX_RINGS; j++) {
				struct ring_sample *rs = sample->ring_samples + j;

                                if (!rings[j].size)
                                        continue;

				rs->ccid_end =
				    ring_read(rings + j, RING_CCID) & CCID_ADDR_MASK;
			}

			tf = gettime();
			if (tf - t1 >= 1000000) {
				/* We are out of sync, bail out */
				last_samples_per_sec = i+1;
				break;
			}
			interval = def_sleep - (tf - ti);
			if (interval > 0)
				usleep(interval);
		}

		for (i = 0; i < MAX_RINGS; i++) {
			struct ring *ring = rings + i;

                        if (!ring->size)
                                continue;

			ring->full = 0;
			ring->idle = 0;
			ring->n_samples = 0;
		}
		for (i = 0; i < num_instdone_bits; i++)
			top_bits[i].count = 0;

                n_contexts = analyse_samples(devid, samples, last_samples_per_sec);
                if (!n_contexts) {
                        fprintf(stderr, "Not able to distinguish even one "
                                "context in samples!");
                        exit(1);
                }

		qsort(top_bits_sorted, num_instdone_bits,
		      sizeof(struct top_bit *), top_bits_sort);
		qsort(contexts_sorted, MAX_CONTEXTS,
		      sizeof(struct context *), contexts_sort);

		/* Limit the number of lines printed to the terminal height so the
		 * most important info (at the top) will stay on screen. */
		max_lines = -1;
		if (ioctl(0, TIOCGWINSZ, &ws) != -1)
			max_lines = ws.ws_row - 6; /* exclude header lines */

		t2 = gettime();
		elapsed_time += (t2 - t1) / 1000000.0;

		if (interactive) {
			int ctx_i = 0;
			int stat_i = -1; /* account for context header */
			int percent;

			printf("%s", clear_screen);
			print_clock_info(pci_dev);

			for (i = 0; i < MAX_RINGS; i++) {
                                if (!rings[i].size)
                                        continue;
				ring_print(rings + i);
                        }

			printf("\n%30s  %s\n", "task", "percent busy");
			for (i = 0; i < max_lines; i++) {
				if (i < num_instdone_bits &&
				    top_bits_sorted[i]->count > 0) {
					percent = (top_bits_sorted[i]->count * 100) /
						last_samples_per_sec;
					len = printf("%30s: %3d%%: ",
							 top_bits_sorted[i]->bit->name,
							 percent);
					print_percentage_bar (percent, len);
				} else {
					printf("%*s", PERCENTAGE_BAR_END, "");
				}

				if (ctx_i < n_contexts && HAS_STATS_REGS(devid)) {
					struct context *context = contexts_sorted[ctx_i];

					if (stat_i == -1) {
						percent = (context->n_samples * 100) /
							last_samples_per_sec;
						printf("context = %" PRIx32 " : %d%% active",
						       context->id, percent);
					} else if (!context->stats[stat_i].warped) {
						printf("   %-15s: (%" PRIu64 "/sec)",
						       stats_reg_names[stat_i],
						       (context->stats[stat_i].end -
							context->stats[stat_i].start));
					} else {
						printf("   %-15s: %" PRIu64 " (Time Warp Error)",
						       stats_reg_names[stat_i],
						       context->stats[stat_i].end);
					}
					if (++stat_i == STATS_COUNT) {
					    ctx_i++;
					    stat_i = -1;
					}
				} else {
					if (i >= num_instdone_bits ||
					    !top_bits_sorted[i]->count)
						break;
				}
				printf("\n");
			}
		}
		if (output) {
			/* Print headers for columns at first run */
			if (print_headers) {
				fprintf(output, "#%15s %10s", "time", "context");
				for (i = 0; i < MAX_RINGS; i++) {
                                        if (!rings[i].size)
                                                continue;
					ring_print_header(output, &rings[i]);
                                }
				for (i = 0; i < STATS_COUNT; i++) {
					fprintf(output, " %15s", stats_reg_names[i]);
				}
				fprintf(output, "\n");
				print_headers = 0;
			}

			/* Print statistics */
			fprintf(output, " %15.2f %10s", elapsed_time, "");

			for (i = 0; i < MAX_RINGS; i++) {
                                if (!rings[i].size)
                                        continue;
				ring_log(&rings[i], output);
                        }
			fprintf(output, "\n");
			for (i = 0; i < n_contexts; i++) {
				struct context *context = contexts_sorted[i];

				fprintf(output, " %15s %10" PRIx32, "", context->id);

				for (int j = 0; j < MAX_RINGS; j++) {
                                        if (!rings[i].size)
                                                continue;
					fprintf(output, " %10s %6s", "", "");
                                }

				for (int j = 0; j < STATS_COUNT; j++) {
					fprintf(output, " %15" PRIu64,
						(context->stats[j].end -
						 context->stats[j].start));
				}
				fprintf(output, "\n");
			}
			fprintf(output, "\n");
			fflush(output);
		}

		/* Check if child has gone */
		if (child_pid > 0) {
			int res;
			if ((res = waitpid(child_pid, &child_stat, WNOHANG)) == -1) {
				perror("waitpid");
				exit(1);
			}
			if (res == 0)
				continue;
			if (WIFEXITED(child_stat))
				break;
		}
	}

        if (output)
                fclose(output);

	intel_register_access_fini();
	return 0;
}
