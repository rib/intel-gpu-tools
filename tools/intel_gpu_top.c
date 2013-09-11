/*
 * Copyright © 2007-2013 Intel Corporation
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
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#ifdef HAVE_TERMIOS_H
#include <termios.h>
#endif
#include <sys/ioctl.h>

#include <linux/perf_event.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "intel_io.h"
#include "intel_chipset.h"
#include "instdone.h"
#include "i915_perf.h"

#define MAX_NUM_TOP_BITS            100

static struct top_bit {
	struct instdone_bit *bit;
	uint64_t count;
} top_bits[MAX_NUM_TOP_BITS], *top_bits_sorted[MAX_NUM_TOP_BITS];

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

static const char *statistic_names[] = {
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

static uint64_t i915_type_id(void)
{
	static uint64_t id;

	if (id == 0) {
		char buf[1024];
		int fd, n;

		fd = open("/sys/bus/event_source/devices/i915/type", 0);
		if (fd < 0) {
			n = -1;
		} else {
			n = read(fd, buf, sizeof(buf)-1);
			close(fd);
		}
		if (n < 0)
			return 0;

		buf[n] = '\0';
		id = strtoull(buf, 0, 0);
	}

	return id;
}

static int perf_i915_open(int config, int group)
{
	struct perf_event_attr attr;

	memset(&attr, 0, sizeof (attr));

	attr.type = i915_type_id();
	if (attr.type == 0)
		return -ENOENT;
	attr.config = config;

	attr.read_format = PERF_FORMAT_TOTAL_TIME_ENABLED;
	if (group == -1)
		attr.read_format |= PERF_FORMAT_GROUP;

	return perf_event_open(&attr, -1, 0, group, 0);
}

static int num_rings, have_wait, have_sema;
static int have_act_freq, have_req_freq, have_power;
static int have_instdone, have_statistic;

static int perf_init(void)
{
	int fd, n;

	fd = perf_i915_open(I915_PERF_RING_BUSY(0), -1);
	if (fd < 0)
		return -1;

	if (perf_i915_open(I915_PERF_RING_WAIT(0), fd) >= 0)
		have_wait = 1;

	if (perf_i915_open(I915_PERF_RING_SEMA(0), fd) >= 0)
		have_sema = 1;

	for (n = 1; n < 4; n++) {
		if (perf_i915_open(I915_PERF_RING_BUSY(n), fd) < 0)
			break;

		if (have_wait && perf_i915_open(I915_PERF_RING_WAIT(n), fd) < 0)
			return -1;

		if (have_sema && perf_i915_open(I915_PERF_RING_SEMA(n), fd) < 0)
			return -1;
	}
	num_rings = n;

	have_act_freq = perf_i915_open(I915_PERF_ACTUAL_FREQUENCY, fd) >= 0;
	have_req_freq = perf_i915_open(I915_PERF_REQUESTED_FREQUENCY, fd) >= 0;
	have_power = perf_i915_open(I915_PERF_ENERGY, fd) >= 0;

	for (n = 0; n < num_instdone_bits; n++) {
		if (perf_i915_open(I915_PERF_INSTDONE(n), fd) < 0)
			break;
	}
	have_instdone = n;

	for (n = 0; n < sizeof(statistic_names)/sizeof(statistic_names[0]); n++) {
		if (perf_i915_open(I915_PERF_STATISTIC(n), fd) < 0)
			break;
	}
	have_statistic = n;

	return fd;
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

static int clamp_pc(int v)
{
	if (v < 0)
		return 0;
	if (v > 100)
		return 100;
	return v;
}

static void ring_print(int ring, uint64_t d_time, uint64_t d_busy, uint64_t d_wait, uint64_t d_sema)
{
	const char *ring_name[] = { "RCS", "VCS", "BCS", "VECS" };
	int percent_busy, len;

	percent_busy = clamp_pc(100 * d_busy / d_time);

	len = printf("%25s busy: %3d%%: ", ring_name[ring], percent_busy);
	print_percentage_bar (percent_busy, len);

	printf("%24s ", ring_name[ring]);
	if (d_wait)
		printf(" wait: %d", clamp_pc(100 * d_wait / d_time));
	if (d_sema)
		printf(" sema: %d", clamp_pc(100 * d_sema / d_time));
	printf("\n");
}

static void
usage(const char *appname)
{
	printf("intel-gpu-top - Display a top-like summary of Intel GPU usage\n"
	       "\n"
	       "usage: %s [parameters]\n"
	       "\n"
	       "The following parameters apply:\n"
	       "[-h]                 show this help screen\n"
	       "\n",
	       appname);
}

int main(int argc, char **argv)
{
	uint64_t data[2][128];
	int sample_count = 0;
	int fd, i, n;

	/* Parse options? */
	while ((i = getopt(argc, argv, "h")) != -1) {
		switch (i) {
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

	init_instdone_definitions(intel_get_pci_device()->device_id);

	fd = perf_init();
	if (fd < 0 || read(fd, data[0], sizeof(data[0])) < 0) {
		fprintf(stderr, "Failed to open i915 PMU\n");
		exit(ENODEV);
	}

	for (i = 0; i < num_instdone_bits; i++) {
		top_bits[i].bit = &instdone_bits[i];
		top_bits[i].count = 0;
		top_bits_sorted[i] = &top_bits[i];
	}

	while (1) {
		struct winsize ws;
		uint64_t *this, *last;
		uint64_t d_time;
		int max_lines;

		last = data[sample_count&1];
		this = data[++sample_count&1];

		usleep(100000);
		read(fd, this, sizeof(data[0]));

		/* Limit the number of lines printed to the terminal height so the
		 * most important info (at the top) will stay on screen. */
		max_lines = -1;
		if (ioctl(0, TIOCGWINSZ, &ws) != -1)
			max_lines = ws.ws_row; /* exclude header lines */

		puts("\e[H\e[J");

		d_time = this[1] - last[1];

		i = 2;
		for (n = 0; n < num_rings; n++) {
			uint64_t busy, wait, sema;

			busy = this[i] - last[i];
			wait = have_wait ? ++i, this[i] - last[i] : 0;
			sema = have_sema ? ++i, this[i] - last[i] : 0;
			ring_print(n, d_time, busy, wait, sema);
			max_lines--;
			i++;
		}

		{
			uint64_t power, actfreq, reqfreq;

			i--;
			actfreq = have_act_freq ? ++i, this[i] - last[i] : 0;
			reqfreq = have_req_freq ? ++i, this[i] - last[i] : 0;
			power = have_power ? ++i, this[i] - last[i] : 0;
			i++;

			if (actfreq) {
				printf("\nFrequency: %dMhz", (int)(actfreq / d_time));
				if (reqfreq)
					printf(" (requested %dMhz)", (int)(actfreq / d_time));
				if (power)
					printf(", power: %dmW", (int)(1000*1000*power / d_time));
				printf("\n");
				max_lines -= 2;
			}
		}

		for (n = 0; n < have_instdone; n++) {
			top_bits[n].count = this[i] - last[i];
			i++;
		}
		qsort(top_bits_sorted, num_instdone_bits,
		      sizeof(struct top_bit *), top_bits_sort);

		if (--max_lines >= num_instdone_bits)
			max_lines = num_instdone_bits;

		printf("\n%30s  %s\n", "task", "percent busy");
		for (n = 0; n < max_lines; n++) {
			if (top_bits_sorted[n]->count > 0) {
				int percent = clamp_pc(100 * top_bits_sorted[n]->count / d_time);
				int len = printf("%30s: %3d%%: ",
						 top_bits_sorted[n]->bit->name,
						 percent);
				print_percentage_bar (percent, len);
			} else {
				printf("%*s", PERCENTAGE_BAR_END, "");
			}

			if (n < have_statistic) {
				printf("%13s: %llu (%lld/sec)",
				       statistic_names[n],
				       (long long)this[n+i],
				       (long long)(this[n+i] - last[n+i]) / d_time);
			}
			printf("\n");

			if (top_bits_sorted[n]->count == 0 && n >= have_statistic)
				break;
		}
	}

	return 0;
}
