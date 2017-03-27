/*
 * Copyright © 2016 Intel Corporation
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <inttypes.h>
#include <errno.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <dirent.h>
#include <time.h>
#include <poll.h>
#include <math.h>

#include "igt.h"
#include "drm.h"

IGT_TEST_DESCRIPTION("Test the i915 perf metrics streaming interface");

#define GEN6_MI_REPORT_PERF_COUNT ((0x28 << 23) | (3 - 2))
#define GEN8_MI_REPORT_PERF_COUNT ((0x28 << 23) | (4 - 2))

#define OAREPORT_REASON_MASK           0x3f
#define OAREPORT_REASON_SHIFT          19
#define OAREPORT_REASON_TIMER          (1<<0)
#define OAREPORT_REASON_CTX_SWITCH     (1<<3)
#define OAREPORT_REASON_CLK_RATIO      (1<<5)

#define GFX_OP_PIPE_CONTROL     ((3 << 29) | (3 << 27) | (2 << 24))
#define PIPE_CONTROL_CS_STALL	   (1 << 20)
#define PIPE_CONTROL_GLOBAL_SNAPSHOT_COUNT_RESET	(1 << 19)
#define PIPE_CONTROL_TLB_INVALIDATE     (1 << 18)
#define PIPE_CONTROL_SYNC_GFDT	  (1 << 17)
#define PIPE_CONTROL_MEDIA_STATE_CLEAR  (1 << 16)
#define PIPE_CONTROL_NO_WRITE	   (0 << 14)
#define PIPE_CONTROL_WRITE_IMMEDIATE    (1 << 14)
#define PIPE_CONTROL_WRITE_DEPTH_COUNT  (2 << 14)
#define PIPE_CONTROL_WRITE_TIMESTAMP    (3 << 14)
#define PIPE_CONTROL_DEPTH_STALL	(1 << 13)
#define PIPE_CONTROL_RENDER_TARGET_FLUSH (1 << 12)
#define PIPE_CONTROL_INSTRUCTION_INVALIDATE (1 << 11)
#define PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE   (1 << 10) /* GM45+ only */
#define PIPE_CONTROL_ISP_DIS	    (1 << 9)
#define PIPE_CONTROL_INTERRUPT_ENABLE   (1 << 8)
#define PIPE_CONTROL_FLUSH_ENABLE       (1 << 7) /* Gen7+ only */
/* GT */
#define PIPE_CONTROL_DATA_CACHE_INVALIDATE      (1 << 5)
#define PIPE_CONTROL_VF_CACHE_INVALIDATE	(1 << 4)
#define PIPE_CONTROL_CONST_CACHE_INVALIDATE     (1 << 3)
#define PIPE_CONTROL_STATE_CACHE_INVALIDATE     (1 << 2)
#define PIPE_CONTROL_STALL_AT_SCOREBOARD	(1 << 1)
#define PIPE_CONTROL_DEPTH_CACHE_FLUSH	  (1 << 0)
#define PIPE_CONTROL_PPGTT_WRITE	(0 << 2)
#define PIPE_CONTROL_GLOBAL_GTT_WRITE   (1 << 2)

/* Temporarily copy i915-perf uapi here to avoid a dependency on libdrm's
 * i915_drm.h copy being updated with the i915-perf interface before this
 * test can land in i-g-t.
 *
 * TODO: remove this once the interface lands in libdrm
 */
#ifndef DRM_I915_PERF_OPEN
#define DRM_I915_PERF_OPEN		0x36
#define DRM_IOCTL_I915_PERF_OPEN	DRM_IOW(DRM_COMMAND_BASE + DRM_I915_PERF_OPEN, struct drm_i915_perf_open_param)

enum drm_i915_oa_format {
	I915_OA_FORMAT_A13 = 1,     /* HSW only */
	I915_OA_FORMAT_A29,         /* HSW only */
	I915_OA_FORMAT_A13_B8_C8,   /* HSW only */
	I915_OA_FORMAT_B4_C8,       /* HSW only */
	I915_OA_FORMAT_A45_B8_C8,   /* HSW only */
	I915_OA_FORMAT_B4_C8_A16,   /* HSW only */
	I915_OA_FORMAT_C4_B8,       /* HSW+ */

	/* Gen8+ */
	I915_OA_FORMAT_A12,
	I915_OA_FORMAT_A12_B8_C8,
	I915_OA_FORMAT_A32u40_A4u32_B8_C8,

	I915_OA_FORMAT_MAX /* non-ABI */
};

enum drm_i915_perf_property_id {
       DRM_I915_PERF_PROP_CTX_HANDLE = 1,
       DRM_I915_PERF_PROP_SAMPLE_OA,
       DRM_I915_PERF_PROP_OA_METRICS_SET,
       DRM_I915_PERF_PROP_OA_FORMAT,
       DRM_I915_PERF_PROP_OA_EXPONENT,

       DRM_I915_PERF_PROP_MAX /* non-ABI */
};

struct drm_i915_perf_open_param {
       __u32 flags;
#define I915_PERF_FLAG_FD_CLOEXEC	(1<<0)
#define I915_PERF_FLAG_FD_NONBLOCK	(1<<1)
#define I915_PERF_FLAG_DISABLED		(1<<2)

       __u32 num_properties;
       __u64 properties_ptr;
};

#define I915_PERF_IOCTL_ENABLE _IO('i', 0x0)
#define I915_PERF_IOCTL_DISABLE	_IO('i', 0x1)

struct drm_i915_perf_record_header {
       __u32 type;
       __u16 pad;
       __u16 size;
};

enum drm_i915_perf_record_type {
       DRM_I915_PERF_RECORD_SAMPLE = 1,
       DRM_I915_PERF_RECORD_OA_REPORT_LOST = 2,
       DRM_I915_PERF_RECORD_OA_BUFFER_LOST = 3,

       DRM_I915_PERF_RECORD_MAX /* non-ABI */
};
#endif /* !DRM_I915_PERF_OPEN */

struct counters_record {
	enum drm_i915_oa_format format;

	uint64_t *a_records;
	uint64_t *a40_records;
	uint64_t *b_records;
	uint64_t *c_records;
};

static struct {
	const char *name;
	size_t size;
	int a40_high_off; /* bytes */
	int a40_low_off;
	int n_a40;
	int a_off;
	int n_a;
	int first_a;
	int b_off;
	int n_b;
	int c_off;
	int n_c;
	int min_gen;
	int max_gen;
} oa_formats[I915_OA_FORMAT_MAX] = {
	[I915_OA_FORMAT_A13] = { /* HSW only */
		"A13", .size = 64,
		.a_off = 12, .n_a = 13,
		.max_gen = 7 },
	[I915_OA_FORMAT_A29] = { /* HSW only */
		"A29", .size = 128,
		.a_off = 12, .n_a = 29,
		.max_gen = 7 },
	[I915_OA_FORMAT_A13_B8_C8] = { /* HSW only */
		"A13_B8_C8", .size = 128,
		.a_off = 12, .n_a = 13,
		.b_off = 64, .n_b = 8,
		.c_off = 96, .n_c = 8,
		.max_gen = 7 },
	[I915_OA_FORMAT_A45_B8_C8] = { /* HSW only */
		"A45_B8_C8", .size = 256,
		.a_off = 12,  .n_a = 45,
		.b_off = 192, .n_b = 8,
		.c_off = 224, .n_c = 8,
		.max_gen = 7 },
	[I915_OA_FORMAT_B4_C8] = { /* HSW only */
		"B4_C8", .size = 64,
		.b_off = 16, .n_b = 4,
		.c_off = 32, .n_c = 8,
		.max_gen = 7 },
	[I915_OA_FORMAT_B4_C8_A16] = { /* HSW only */
		"B4_C8_A16", .size = 128,
		.b_off = 16, .n_b = 4,
		.c_off = 32, .n_c = 8,
		.a_off = 60, .n_a = 16, .first_a = 29,
		.max_gen = 7 },
	[I915_OA_FORMAT_C4_B8] = { /* HSW+ (header differs from HSW-Gen8+) */
		"C4_B8", .size = 64,
		.c_off = 16, .n_c = 4,
		.b_off = 28, .n_b = 8 },

	/* Gen8+ */

	[I915_OA_FORMAT_A12] = {
		"A12", .size = 64,
		.a_off = 12, .n_a = 12, .first_a = 7,
		.min_gen = 8 },
	[I915_OA_FORMAT_A12_B8_C8] = {
		"A12_B8_C8", .size = 128,
		.a_off = 12, .n_a = 12,
		.b_off = 64, .n_b = 8,
		.c_off = 96, .n_c = 8, .first_a = 7,
		.min_gen = 8 },
	[I915_OA_FORMAT_A32u40_A4u32_B8_C8] = {
		"A32u40_A4u32_B8_C8", .size = 256,
		.a40_high_off = 160, .a40_low_off = 16, .n_a40 = 32,
		.a_off = 144, .n_a = 4, .first_a = 32,
		.b_off = 192, .n_b = 8,
		.c_off = 224, .n_c = 8,
		.min_gen = 8 },
	[I915_OA_FORMAT_C4_B8] = {
		"C4_B8", .size = 64,
		.c_off = 16, .n_c = 4,
		.b_off = 32, .n_b = 8,
		.min_gen = 8 },
};

static bool hsw_undefined_a_counters[45] = {
	[4] = true,
	[6] = true,
	[9] = true,
	[11] = true,
	[14] = true,
	[16] = true,
	[19] = true,
	[21] = true,
	[24] = true,
	[26] = true,
	[29] = true,
	[31] = true,
	[34] = true,
	[43] = true,
	[44] = true,
};

/* No A counters currently reserved/undefined for gen8+ so far */
static bool gen8_undefined_a_counters[45];

static int drm_fd = -1;
static int stream_fd = -1;
static uint32_t devid;
static int device = -1;
static int n_eus;

static uint64_t test_metric_set_id = UINT64_MAX;
static uint64_t gt_min_freq_mhz_saved = 0;
static uint64_t gt_max_freq_mhz_saved = 0;
static uint64_t gt_min_freq_mhz = 0;
static uint64_t gt_max_freq_mhz = 0;

static uint64_t timestamp_frequency = 12500000;
static enum drm_i915_oa_format test_oa_format;
static bool *undefined_a_counters;
static uint64_t oa_exp_1_millisec;

static igt_render_copyfunc_t render_copy = NULL;
static uint32_t (*read_report_ticks)(uint32_t *report,
				     enum drm_i915_oa_format format);
static void (*sanity_check_reports)(uint32_t *oa_report0, uint32_t *oa_report1,
				    enum drm_i915_oa_format format);

static void
__perf_close(int fd)
{
	close(fd);
	stream_fd = -1;
}

static int
__perf_open(int fd, struct drm_i915_perf_open_param *param)
{
	int ret;

	if (stream_fd >= 0)
		__perf_close(stream_fd);

	ret = igt_ioctl(fd, DRM_IOCTL_I915_PERF_OPEN, param);

	igt_assert(ret >= 0);
	errno = 0;

	return ret;
}

static int
lookup_format(int i915_perf_fmt_id)
{
	igt_assert(i915_perf_fmt_id < I915_OA_FORMAT_MAX);
	igt_assert(oa_formats[i915_perf_fmt_id].name);

	return i915_perf_fmt_id;
}

static bool
try_read_u64_file(const char *file, uint64_t *val)
{
	char buf[32];
	int fd, n;

	fd = open(file, O_RDONLY);
	if (fd < 0)
		return false;

	while ((n = read(fd, buf, sizeof(buf) - 1)) < 0 && errno == EINTR)
		;
	igt_assert(n >= 0);

	close(fd);

	buf[n] = '\0';
	*val = strtoull(buf, NULL, 0);

	return true;
}

static uint64_t
read_u64_file(const char *file)
{
	uint64_t val;

	igt_assert_eq(try_read_u64_file(file, &val), true);

	return val;
}

static void
write_u64_file(const char *file, uint64_t val)
{
	char buf[32];
	int fd, len, ret;

	fd = open(file, O_WRONLY);
	igt_assert(fd >= 0);

	len = snprintf(buf, sizeof(buf), "%"PRIu64, val);
	igt_assert(len > 0);

	while ((ret = write(fd, buf, len)) < 0 && errno == EINTR)
		;
	igt_assert_eq(ret, len);

	close(fd);
}

static uint64_t
sysfs_read(const char *file)
{
	char buf[512];

	snprintf(buf, sizeof(buf), "/sys/class/drm/card%d/%s", device, file);

	return read_u64_file(buf);
}

static void
sysfs_write(const char *file, uint64_t val)
{
	char buf[512];

	snprintf(buf, sizeof(buf), "/sys/class/drm/card%d/%s", device, file);

	write_u64_file(buf, val);
}

static char *
read_debugfs_record(const char *file, const char *key)
{
	FILE *fp;
	char *line = NULL;
	size_t line_buf_size = 0;
	int len = 0;
	int key_len = strlen(key);
	char *value = NULL;

	fp = igt_debugfs_fopen(file, "r");
	igt_require(fp);

	while ((len = getline(&line, &line_buf_size, fp)) > 0) {

		if (line[len - 1] == '\n')
			line[len - 1] = '\0';

		if (strncmp(key, line, key_len) == 0 &&
		    line[key_len] == ':' &&
		    line[key_len + 1] == ' ')
		{
			value = strdup(line + key_len + 2);
			goto done;
		}
	}

	igt_assert(!"reached");
done:
	free(line);
	if (fp)
		fclose(fp);
	return value;
}

static uint64_t
read_debugfs_u64_record(const char *file, const char *key)
{
	char *str_val = read_debugfs_record(file, key);
	uint64_t val;

	igt_require(str_val);

	val = strtoull(str_val, NULL, 0);
	free(str_val);

	return val;
}

/* XXX: For Haswell this utility is only applicable to the render basic
 * metric set.
 *
 * C2 corresponds to a clock counter for the Haswell render basic metric set
 * but it's not included in all of the formats.
 */
static uint32_t
hsw_read_report_ticks(uint32_t *report, enum drm_i915_oa_format format)
{
	uint32_t *c = (uint32_t *)(((uint8_t *)report) + oa_formats[format].c_off);

	igt_assert_neq(oa_formats[format].n_c, 0);

	return c[2];
}

static uint32_t
gen8_read_report_ticks(uint32_t *report, enum drm_i915_oa_format format)
{
	return report[3];
}

static uint64_t
timebase_scale(uint32_t u32_delta)
{
	return ((uint64_t)u32_delta * NSEC_PER_SEC) / timestamp_frequency;
}

/* Returns: the largest OA exponent that will still result in a sampling period
 * less than or equal to the given @period.
 */
static int
max_oa_exponent_for_period_lte(uint64_t period)
{
	/* NB: timebase_scale() takes a uint32_t and an exponent of 30
	 * would already represent a period of ~3 minutes so there's
	 * really no need to consider higher exponents.
	 */
	for (int i = 0; i < 30; i++) {
		uint64_t oa_period = timebase_scale(2 << i);

		if (oa_period > period)
			return max(0, i - 1);
	}

	igt_assert(!"reached");
	return -1;
}

/* Return: the largest OA exponent that will still result in a sampling
 * frequency greater than the given @frequency.
 */
static int
max_oa_exponent_for_freq_gt(uint64_t frequency)
{
	uint64_t period = NSEC_PER_SEC / frequency;

	igt_assert_neq(period, 0);

	return max_oa_exponent_for_period_lte(period - 1);
}

static uint64_t
oa_exponent_to_ns(int exponent)
{
       return 1000000000ULL * (2ULL << exponent) / timestamp_frequency;
}

static void
hsw_sanity_check_render_basic_reports(uint32_t *oa_report0, uint32_t *oa_report1,
				      enum drm_i915_oa_format fmt)
{
	uint32_t time_delta = timebase_scale(oa_report1[1] - oa_report0[1]);
	uint32_t clock_delta;
	uint32_t max_delta;

	igt_assert_neq(time_delta, 0);

	/* As a special case we have to consider that on Haswell we
	 * can't explicitly derive a clock delta for all OA report
	 * formats...
	 */
	if (oa_formats[fmt].n_c == 0) {
		/* Assume running at max freq for sake of
		 * below sanity check on counters... */
		clock_delta = (gt_max_freq_mhz *
			       (uint64_t)time_delta) / 1000;
	} else {
		uint32_t ticks0 = read_report_ticks(oa_report0, fmt);
		uint32_t ticks1 = read_report_ticks(oa_report1, fmt);
		uint64_t freq;

		clock_delta = ticks1 - ticks0;

		igt_assert_neq(clock_delta, 0);

		freq = ((uint64_t)clock_delta * 1000) / time_delta;
		igt_debug("freq = %"PRIu64"\n", freq);

		igt_assert(freq <= gt_max_freq_mhz);
	}

	igt_debug("clock delta = %"PRIu32"\n", clock_delta);

	/* The maximum rate for any HSW counter =
	 *   clock_delta * N EUs
	 *
	 * Sanity check that no counters exceed this delta.
	 */
	max_delta = clock_delta * n_eus;

	/* 40bit A counters were only introduced for Gen8+ */
	igt_assert_eq(oa_formats[fmt].n_a40, 0);

	for (int j = 0; j < oa_formats[fmt].n_a; j++) {
		uint32_t *a0 = (uint32_t *)(((uint8_t *)oa_report0) +
					    oa_formats[fmt].a_off);
		uint32_t *a1 = (uint32_t *)(((uint8_t *)oa_report1) +
					    oa_formats[fmt].a_off);
		int a_id = oa_formats[fmt].first_a + j;
		uint32_t delta = a1[j] - a0[j];

		if (undefined_a_counters[a_id])
			continue;

		igt_debug("A%d: delta = %"PRIu32"\n", a_id, delta);
		igt_assert(delta <= max_delta);
	}

	for (int j = 0; j < oa_formats[fmt].n_b; j++) {
		uint32_t *b0 = (uint32_t *)(((uint8_t *)oa_report0) +
					    oa_formats[fmt].b_off);
		uint32_t *b1 = (uint32_t *)(((uint8_t *)oa_report1) +
					    oa_formats[fmt].b_off);
		uint32_t delta = b1[j] - b0[j];

		igt_debug("B%d: delta = %"PRIu32"\n", j, delta);
		igt_assert(delta <= max_delta);
	}

	for (int j = 0; j < oa_formats[fmt].n_c; j++) {
		uint32_t *c0 = (uint32_t *)(((uint8_t *)oa_report0) +
					    oa_formats[fmt].c_off);
		uint32_t *c1 = (uint32_t *)(((uint8_t *)oa_report1) +
					    oa_formats[fmt].c_off);
		uint32_t delta = c1[j] - c0[j];

		igt_debug("C%d: delta = %"PRIu32"\n", j, delta);
		igt_assert(delta <= max_delta);
	}
}

static uint64_t
gen8_read_40bit_a_counter(uint32_t *report, enum drm_i915_oa_format fmt, int a_id)
{
	uint8_t *a40_high = (((uint8_t *)report) + oa_formats[fmt].a40_high_off);
	uint32_t *a40_low = (uint32_t *)(((uint8_t *)report) +
					 oa_formats[fmt].a40_low_off);
	uint64_t high = (uint64_t)(a40_high[a_id]) << 32;

	return a40_low[a_id] | high;
}

static uint64_t
gen8_40bit_a_delta(uint64_t value0, uint64_t value1)
{
	if (value0 > value1)
		return (1ULL << 40) + value1 - value0;
	else
		return value1 - value0;
}

/**/
static struct counters_record *
counters_record_allocate(enum drm_i915_oa_format format)
{
	struct counters_record *record = calloc(1, sizeof(struct counters_record));
	record->format = format;

	record->a_records = calloc(oa_formats[format].n_a, sizeof(uint64_t));
	record->a40_records = calloc(oa_formats[format].n_a40, sizeof(uint64_t));
	record->b_records = calloc(oa_formats[format].n_b, sizeof(uint64_t));
	record->c_records = calloc(oa_formats[format].n_c, sizeof(uint64_t));

	return record;
}

static void
counters_record_free(struct counters_record *record)
{
	free(record->a_records);
	free(record->a40_records);
	free(record->b_records);
	free(record->c_records);
	free(record);
}

static void
counters_record_update(struct counters_record *record,
		       uint32_t *oa_report0, uint32_t *oa_report1)
{
	for (uint32_t i = 0; i < oa_formats[record->format].n_a; i++) {
		uint32_t *oa_report0_data = (uint32_t *)
			(((uint8_t *) oa_report0) + oa_formats[record->format].a_off),
			*oa_report1_data = (uint32_t *)
			(((uint8_t *) oa_report1) + oa_formats[record->format].a_off);
		record->a_records[i] += oa_report1_data[i] - oa_report0_data[i];
	}

	for (uint32_t i = 0; i < oa_formats[record->format].n_a40; i++) {
		uint64_t value0 = gen8_read_40bit_a_counter(oa_report0, record->format, i),
			value1 = gen8_read_40bit_a_counter(oa_report1, record->format, i);
		record->a40_records[i] += gen8_40bit_a_delta(value0, value1);
	}
}

static void
counters_record_reset(struct counters_record *record)
{
	memset(record->a_records, 0, sizeof(record->a_records[0]) * oa_formats[record->format].n_a);
	memset(record->a40_records, 0, sizeof(record->a40_records[0]) * oa_formats[record->format].n_a40);
	memset(record->b_records, 0, sizeof(record->b_records[0]) * oa_formats[record->format].n_b);
	memset(record->c_records, 0, sizeof(record->c_records[0]) * oa_formats[record->format].n_c);
}

static void
counters_record_print(struct counters_record *record, const char *title)
{
	igt_debug("%s:\n", title);
	for (uint32_t i = 0; i < oa_formats[record->format].n_a; i++)
		igt_debug("\tA%u = %lu\n", i, record->a_records[i]);
	for (uint32_t i = 0; i < oa_formats[record->format].n_a40; i++)
		igt_debug("\tA40_%u = %lu\n", i, record->a40_records[i]);
}

/* The TestOa metric set is designed so */
static void
gen8_sanity_check_test_oa_reports(uint32_t *oa_report0, uint32_t *oa_report1,
				  enum drm_i915_oa_format fmt)
{
	uint32_t time_delta = timebase_scale(oa_report1[1] - oa_report0[1]);
	uint32_t ticks0 = read_report_ticks(oa_report0, fmt);
	uint32_t ticks1 = read_report_ticks(oa_report1, fmt);
	uint32_t clock_delta = ticks1 - ticks0;
	uint32_t max_delta;
	uint64_t freq;
	uint32_t *rpt0_b = (uint32_t *)(((uint8_t *)oa_report0) +
					oa_formats[fmt].b_off);
	uint32_t *rpt1_b = (uint32_t *)(((uint8_t *)oa_report1) +
					oa_formats[fmt].b_off);
	uint32_t b;
	uint32_t ref;


	igt_assert_neq(time_delta, 0);
	igt_assert_neq(clock_delta, 0);

	freq = ((uint64_t)clock_delta * 1000) / time_delta;
	igt_debug("freq = %"PRIu64"\n", freq);

	igt_assert(freq <= gt_max_freq_mhz);

	igt_debug("clock delta = %"PRIu32"\n", clock_delta);

	max_delta = clock_delta * n_eus;

	/* Gen8+ has some 40bit A counters... */
	for (int j = 0; j < oa_formats[fmt].n_a40; j++) {
		uint64_t value0 = gen8_read_40bit_a_counter(oa_report0, fmt, j);
		uint64_t value1 = gen8_read_40bit_a_counter(oa_report1, fmt, j);
		uint64_t delta = gen8_40bit_a_delta(value0, value1);

		if (undefined_a_counters[j])
			continue;

		igt_debug("A%d: delta = %"PRIu64"\n", j, delta);
		igt_assert(delta <= max_delta);
	}

	for (int j = 0; j < oa_formats[fmt].n_a; j++) {
		uint32_t *a0 = (uint32_t *)(((uint8_t *)oa_report0) +
					    oa_formats[fmt].a_off);
		uint32_t *a1 = (uint32_t *)(((uint8_t *)oa_report1) +
					    oa_formats[fmt].a_off);
		int a_id = oa_formats[fmt].first_a + j;
		uint32_t delta = a1[j] - a0[j];

		if (undefined_a_counters[a_id])
			continue;

		igt_debug("A%d: delta = %"PRIu32"\n", a_id, delta);
		igt_assert(delta <= max_delta);
	}

	/* The TestOa metric set defines all B counters to be a
	 * multiple of the gpu clock
	 */
	if (oa_formats[fmt].n_b) {
		b = rpt1_b[0] - rpt0_b[0];
		igt_debug("B0: delta = %"PRIu32"\n", b);
		igt_assert_eq(b, 0);

		b = rpt1_b[1] - rpt0_b[1];
		igt_debug("B1: delta = %"PRIu32"\n", b);
		igt_assert_eq(b, clock_delta);

		b = rpt1_b[2] - rpt0_b[2];
		igt_debug("B2: delta = %"PRIu32"\n", b);
		igt_assert_eq(b, clock_delta);

		b = rpt1_b[3] - rpt0_b[3];
		ref = clock_delta / 2;
		igt_debug("B3: delta = %"PRIu32"\n", b);
		igt_assert(b >= ref - 1 && b <= ref + 1);

		b = rpt1_b[4] - rpt0_b[4];
		ref = clock_delta / 3;
		igt_debug("B4: delta = %"PRIu32"\n", b);
		igt_assert(b >= ref - 1 && b <= ref + 1);

		b = rpt1_b[5] - rpt0_b[5];
		ref = clock_delta / 3;
		igt_debug("B5: delta = %"PRIu32"\n", b);
		igt_assert(b >= ref - 1 && b <= ref + 1);

		b = rpt1_b[6] - rpt0_b[6];
		ref = clock_delta / 6;
		igt_debug("B6: delta = %"PRIu32"\n", b);
		igt_assert(b >= ref - 1 && b <= ref + 1);

		b = rpt1_b[7] - rpt0_b[7];
		ref = clock_delta * 2 / 3;
		igt_debug("B7: delta = %"PRIu32"\n", b);
		igt_assert(b >= ref - 1 && b <= ref + 1);
	}

	for (int j = 0; j < oa_formats[fmt].n_c; j++) {
		uint32_t *c0 = (uint32_t *)(((uint8_t *)oa_report0) +
					    oa_formats[fmt].c_off);
		uint32_t *c1 = (uint32_t *)(((uint8_t *)oa_report1) +
					    oa_formats[fmt].c_off);
		uint32_t delta = c1[j] - c0[j];

		igt_debug("C%d: delta = %"PRIu32"\n", j, delta);
		igt_assert(delta <= max_delta);
	}
}

static bool
init_sys_info(void)
{
	const char *test_set_name = NULL;
	const char *test_set_uuid = NULL;
	char buf[256];

	igt_assert_neq(device, -1);
	igt_assert_neq(devid, 0);

	timestamp_frequency = 12500000;

	if (IS_HASWELL(devid)) {
		/* We don't have a TestOa metric set for Haswell so use
		 * RenderBasic
		 */
		test_set_name = "RenderBasic";
		test_set_uuid = "403d8832-1a27-4aa6-a64e-f5389ce7b212";
		test_oa_format = I915_OA_FORMAT_A45_B8_C8;
		undefined_a_counters = hsw_undefined_a_counters;
		read_report_ticks = hsw_read_report_ticks;
		sanity_check_reports = hsw_sanity_check_render_basic_reports;

		if (intel_gt(devid) == 0)
			n_eus = 10;
		else if (intel_gt(devid) == 1)
			n_eus = 20;
		else if (intel_gt(devid) == 2)
			n_eus = 40;
		else {
			igt_assert(!"reached");
			return false;
		}
	} else {
		drm_i915_getparam_t gp;

		test_set_name = "TestOa";
		test_oa_format = I915_OA_FORMAT_A32u40_A4u32_B8_C8;
		undefined_a_counters = gen8_undefined_a_counters;
		read_report_ticks = gen8_read_report_ticks;
		sanity_check_reports = gen8_sanity_check_test_oa_reports;

		if (IS_BROADWELL(devid)) {
			test_set_uuid = "d6de6f55-e526-4f79-a6a6-d7315c09044e";
		} else if (IS_CHERRYVIEW(devid)) {
			test_set_uuid = "4a534b07-cba3-414d-8d60-874830e883aa";
		} else if (IS_SKYLAKE(devid)) {
			switch (intel_gt(devid)) {
			case 1:
				test_set_uuid = "1651949f-0ac0-4cb1-a06f-dafd74a407d1";
				break;
			case 2:
				test_set_uuid = "2b985803-d3c9-4629-8a4f-634bfecba0e8";
				break;
			case 3:
				test_set_uuid = "882fa433-1f4a-4a67-a962-c741888fe5f5";
				break;
			default:
				igt_debug("unsupport Skylake GT size\n");
				return false;
			}
			timestamp_frequency = 12000000;
		} else if (IS_BROXTON(devid)) {
			test_set_uuid = "5ee72f5c-092f-421e-8b70-225f7c3e9612";
			timestamp_frequency = 19200123;
		} else
			return false;

		gp.param = I915_PARAM_EU_TOTAL;
		gp.value = &n_eus;
		do_ioctl(drm_fd, DRM_IOCTL_I915_GETPARAM, &gp);
	}

	igt_debug("%s metric set UUID = %s\n",
		  test_set_name,
		  test_set_uuid);

	oa_exp_1_millisec = max_oa_exponent_for_period_lte(1000000);

	snprintf(buf, sizeof(buf),
		 "/sys/class/drm/card%d/metrics/%s/id",
		 device,
		 test_set_uuid);

	return try_read_u64_file(buf, &test_metric_set_id);
}

static void
gt_frequency_range_save(void)
{
	gt_min_freq_mhz_saved = sysfs_read("gt_min_freq_mhz");
	gt_max_freq_mhz_saved = sysfs_read("gt_max_freq_mhz");

	gt_min_freq_mhz = gt_min_freq_mhz_saved;
	gt_max_freq_mhz = gt_max_freq_mhz_saved;
}

static void wait_freq_settle(void)
{
	struct timespec ts;

	/* FIXME: Lazy sleep without check. */
	ts.tv_sec = 0;
	ts.tv_nsec = 20000;
	clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
}

static void
gt_frequency_pin(int gt_freq_mhz)
{
	igt_debug("requesting pinned GT freq = %dmhz\n", gt_freq_mhz);

	if (gt_freq_mhz > gt_max_freq_mhz) {
		sysfs_write("gt_max_freq_mhz", gt_freq_mhz);
		sysfs_write("gt_min_freq_mhz", gt_freq_mhz);
	} else {
		sysfs_write("gt_min_freq_mhz", gt_freq_mhz);
		sysfs_write("gt_max_freq_mhz", gt_freq_mhz);
	}
	gt_min_freq_mhz = gt_freq_mhz;
	gt_max_freq_mhz = gt_freq_mhz;

	wait_freq_settle();
}

static void
gt_frequency_range_restore(void)
{
	igt_debug("restoring GT frequency range: min = %dmhz, max =%dmhz, current: min=%dmhz, max=%dmhz\n",
		  (int)gt_min_freq_mhz_saved,
		  (int)gt_max_freq_mhz_saved,
		  (int)gt_min_freq_mhz,
		  (int)gt_max_freq_mhz);

	/* Assume current min/max are the same */
	if (gt_min_freq_mhz_saved > gt_max_freq_mhz) {
		sysfs_write("gt_max_freq_mhz", gt_max_freq_mhz_saved);
		sysfs_write("gt_min_freq_mhz", gt_min_freq_mhz_saved);
	} else {
		sysfs_write("gt_min_freq_mhz", gt_min_freq_mhz_saved);
		sysfs_write("gt_max_freq_mhz", gt_max_freq_mhz_saved);
	}

	gt_min_freq_mhz = gt_min_freq_mhz_saved;
	gt_max_freq_mhz = gt_max_freq_mhz_saved;
}

static int
i915_read_reports_until_timestamp(enum drm_i915_oa_format oa_format,
				  uint8_t *buf,
				  uint32_t max_size,
				  uint32_t start_timestamp,
				  uint32_t end_timestamp)
{
	size_t format_size = oa_formats[oa_format].size;
	uint32_t last_seen_timestamp = start_timestamp;
	int total_len = 0;

	while (last_seen_timestamp < end_timestamp) {
		int offset, len;

		/* Running out of space. */
		if ((max_size - total_len) < format_size) {
			igt_warn("run out of space before reaching "
				 "end timestamp (%u/%u)\n",
				 last_seen_timestamp, end_timestamp);
			return -1;
		}

		while ((len = read(stream_fd, &buf[total_len],
				   max_size - total_len)) < 0 &&
		       errno == EINTR)
			;

		/* Intentionally return an error. */
		if (len <= 0) {
			if (errno == EAGAIN)
				return total_len;
			else {
				igt_warn("error read OA stream : %i\n", errno);
				return -1;
			}
		}

		offset = total_len;
		total_len += len;

		while (offset < total_len) {
		  const struct drm_i915_perf_record_header *header =
		    (const struct drm_i915_perf_record_header *) &buf[offset];
		  uint32_t *report = (uint32_t *) (header + 1);

		  if (header->type == DRM_I915_PERF_RECORD_SAMPLE)
		    last_seen_timestamp = report[1];

		  offset += header->size;
		}
	}

	return total_len;
}


/* CAP_SYS_ADMIN is required to open system wide metrics, unless the system
 * control parameter dev.i915.perf_stream_paranoid == 0 */
static void
test_system_wide_paranoid(void)
{
	igt_fork(child, 1) {
		uint64_t properties[] = {
			/* Include OA reports in samples */
			DRM_I915_PERF_PROP_SAMPLE_OA, true,

			/* OA unit configuration */
			DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
			DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
			DRM_I915_PERF_PROP_OA_EXPONENT, oa_exp_1_millisec,
		};
		struct drm_i915_perf_open_param param = {
			.flags = I915_PERF_FLAG_FD_CLOEXEC |
				I915_PERF_FLAG_FD_NONBLOCK,
			.num_properties = sizeof(properties) / 16,
			.properties_ptr = to_user_pointer(properties),
		};

		write_u64_file("/proc/sys/dev/i915/perf_stream_paranoid", 1);

		igt_drop_root();

		do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EACCES);
	}

	igt_waitchildren();

	igt_fork(child, 1) {
		uint64_t properties[] = {
			/* Include OA reports in samples */
			DRM_I915_PERF_PROP_SAMPLE_OA, true,

			/* OA unit configuration */
			DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
			DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
			DRM_I915_PERF_PROP_OA_EXPONENT, oa_exp_1_millisec,
		};
		struct drm_i915_perf_open_param param = {
			.flags = I915_PERF_FLAG_FD_CLOEXEC |
				I915_PERF_FLAG_FD_NONBLOCK,
			.num_properties = sizeof(properties) / 16,
			.properties_ptr = to_user_pointer(properties),
		};
		write_u64_file("/proc/sys/dev/i915/perf_stream_paranoid", 0);

		igt_drop_root();

		stream_fd = __perf_open(drm_fd, &param);
		__perf_close(stream_fd);
	}

	igt_waitchildren();

	/* leave in paranoid state */
	write_u64_file("/proc/sys/dev/i915/perf_stream_paranoid", 1);
}

static void
test_invalid_open_flags(void)
{
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exp_1_millisec,
	};
	struct drm_i915_perf_open_param param = {
		.flags = ~0, /* Undefined flag bits set! */
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};

	do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EINVAL);
}

static void
test_invalid_oa_metric_set_id(void)
{
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exp_1_millisec,
		DRM_I915_PERF_PROP_OA_METRICS_SET, UINT64_MAX,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC |
			I915_PERF_FLAG_FD_NONBLOCK,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};

	do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EINVAL);

	properties[ARRAY_SIZE(properties) - 1] = 0; /* ID 0 is also be reserved as invalid */
	do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EINVAL);

	/* Check that we aren't just seeing false positives... */
	properties[ARRAY_SIZE(properties) - 1] = test_metric_set_id;
	stream_fd = __perf_open(drm_fd, &param);
	__perf_close(stream_fd);

	/* There's no valid default OA metric set ID... */
	param.num_properties--;
	do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EINVAL);
}

static void
test_invalid_oa_format_id(void)
{
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exp_1_millisec,
		DRM_I915_PERF_PROP_OA_FORMAT, UINT64_MAX,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC |
			I915_PERF_FLAG_FD_NONBLOCK,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};

	do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EINVAL);

	properties[ARRAY_SIZE(properties) - 1] = 0; /* ID 0 is also be reserved as invalid */
	do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EINVAL);

	/* Check that we aren't just seeing false positives... */
	properties[ARRAY_SIZE(properties) - 1] = test_oa_format;
	stream_fd = __perf_open(drm_fd, &param);
	__perf_close(stream_fd);

	/* There's no valid default OA format... */
	param.num_properties--;
	do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EINVAL);
}

static void
test_missing_sample_flags(void)
{
	uint64_t properties[] = {
		/* No _PROP_SAMPLE_xyz flags */

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exp_1_millisec,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};

	do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EINVAL);
}

static int
read_2_oa_reports(int format_id,
		  int exponent,
		  uint32_t *oa_report0,
		  uint32_t *oa_report1,
		  bool timer_only)
{
	size_t format_size = oa_formats[format_id].size;
	size_t sample_size = (sizeof(struct drm_i915_perf_record_header) +
			      format_size);
	const struct drm_i915_perf_record_header *header;
	uint32_t exponent_mask = (1 << (exponent + 1)) - 1;

	/* Note: we allocate a large buffer so that each read() iteration
	 * should scrape *all* pending records.
	 *
	 * The largest buffer the OA unit supports is 16MB.
	 *
	 * Being sure we are fetching all buffered reports allows us to
	 * potentially throw away / skip all reports whenever we see
	 * a _REPORT_LOST notification as a way of being sure are
	 * measurements aren't skewed by a lost report.
	 *
	 * Note: that is is useful for some tests but also not something
	 * applications would be expected to resort to. Lost reports are
	 * somewhat unpredictable but typically don't pose a problem - except
	 * to indicate that the OA unit may be over taxed if lots of reports
	 * are being lost.
	 */
	int max_reports = (16 * 1024 * 1024) / format_size;
	int buf_size = sample_size * max_reports * 1.5;
	uint8_t *buf = malloc(buf_size);
	int n = 0;

	for (int i = 0; i < 1000; i++) {
		ssize_t len;

		while ((len = read(stream_fd, buf, buf_size)) < 0 &&
		       errno == EINTR)
			;

		igt_assert(len > 0);
		igt_debug("read %d bytes\n", (int)len);

		for (size_t offset = 0; offset < len; offset += header->size) {
			const uint32_t *report;

			header = (void *)(buf + offset);

			igt_assert_eq(header->pad, 0); /* Reserved */

			/* Currently the only test that should ever expect to
			 * see a _BUFFER_LOST error is the buffer_fill test,
			 * otherwise something bad has probably happened...
			 */
			igt_assert_neq(header->type, DRM_I915_PERF_RECORD_OA_BUFFER_LOST);

			/* At high sampling frequencies the OA HW might not be
			 * able to cope with all write requests and will notify
			 * us that a report was lost. We restart our read of
			 * two sequential reports due to the timeline blip this
			 * implies
			 */
			if (header->type == DRM_I915_PERF_RECORD_OA_REPORT_LOST) {
				igt_debug("read restart: OA trigger collision / report lost\n");
				n = 0;

				/* XXX: break, because we don't know where
				 * within the series of already read reports
				 * there could be a blip from the lost report.
				 */
				return EAGAIN;
			}

			/* Currently the only other record type expected is a
			 * _SAMPLE. Notably this test will need updating if
			 * i915-perf is extended in the future with additional
			 * record types.
			 */
			igt_assert_eq(header->type, DRM_I915_PERF_RECORD_SAMPLE);

			igt_assert_eq(header->size, sample_size);

			report = (const void *)(header + 1);

			igt_debug("read report: reason = %x, timestamp = %x, exponent mask=%x\n",
				  report[0], report[1], exponent_mask);

			/* Don't expect zero for timestamps */
			igt_assert_neq(report[1], 0);

			if (timer_only) {
				if (IS_HASWELL(devid)) {
					/* For Haswell we don't have a
					 * documented report reason field
					 * (though empirically report[0] bit 10
					 * does seem to correlate with a timer
					 * trigger reason) so we instead infer
					 * which reports are timer triggered by
					 * checking if the least significant
					 * bits are zero and the exponent bit
					 * is set.
					 */
					if ((report[1] & exponent_mask) != (1 << exponent)) {
						igt_debug("skipping non timer report reason=%x\n",
							  report[0]);

						/* Also assert our hypothesis about the
						 * reason bit...
						 */
						igt_assert_eq(report[0] & (1 << 10), 0);
						continue;
					}
				} else {
					uint32_t reason = ((report[0] >> OAREPORT_REASON_SHIFT) &
							   OAREPORT_REASON_MASK);

					if (!(reason & OAREPORT_REASON_TIMER)) {
						igt_debug("skipping non timer report reason=%x\n",
							  reason);
						continue;
					}
				}
			}

			if (n++ == 0)
				memcpy(oa_report0, report, format_size);
			else {
				memcpy(oa_report1, report, format_size);
				free(buf);
				return 0;
			}
		}
	}

	free(buf);

	igt_assert(!"reached");

	return EIO;
}

static void
open_and_read_2_oa_reports(int format_id,
			   int exponent,
			   uint32_t *oa_report0,
			   uint32_t *oa_report1,
			   bool timer_only)
{
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, format_id,
		DRM_I915_PERF_PROP_OA_EXPONENT, exponent,

	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};

	stream_fd = __perf_open(drm_fd, &param);

	while (read_2_oa_reports(format_id, exponent,
				 oa_report0, oa_report1, timer_only) == EAGAIN)
		;

	__perf_close(stream_fd);
}

static void
gen8_read_report_clock_ratios(uint32_t *report,
			      uint32_t *slice_freq_mhz,
			      uint32_t *unslice_freq_mhz)
{
	uint32_t unslice_freq = report[0] & 0x1ff;
	uint32_t slice_freq_low = (report[0] >> 25) & 0x7f;
	uint32_t slice_freq_high = (report[0] >> 9) & 0x3;
	uint32_t slice_freq = slice_freq_low | (slice_freq_high << 7);

	*slice_freq_mhz = (slice_freq * 16666) / 1000;
	*unslice_freq_mhz = (unslice_freq * 16666) / 1000;
}

static const char *
gen8_read_report_reason(uint32_t *report)
{
	uint32_t reason = ((report[0] >> OAREPORT_REASON_SHIFT) &
			   OAREPORT_REASON_MASK);

	if (reason & (1<<0))
		return "timer";
	else if (reason & (1<<1))
	      return "internal trigger 1";
	else if (reason & (1<<2))
	      return "internal trigger 2";
	else if (reason & (1<<3))
	      return "context switch";
	else if (reason & (1<<4))
	      return "GO 1->0 transition (enter RC6)";
	else if (reason & (1<<5))
		return "[un]slice clock ratio change";
	else
		return "unknown";
}

static void
print_reports(uint32_t *oa_report0, uint32_t *oa_report1, int fmt)
{
	igt_debug("TIMESTAMP: 1st = %"PRIu32", 2nd = %"PRIu32", delta = %"PRIu32"\n",
		  oa_report0[1], oa_report1[1], oa_report1[1] - oa_report0[1]);

	if (IS_HASWELL(devid) && oa_formats[fmt].n_c == 0) {
		igt_debug("CLOCK = N/A\n");
	} else {
		uint32_t clock0 = read_report_ticks(oa_report0, fmt);
		uint32_t clock1 = read_report_ticks(oa_report1, fmt);

		igt_debug("CLOCK: 1st = %"PRIu32", 2nd = %"PRIu32", delta = %"PRIu32"\n",
			  clock0, clock1, clock1 - clock0);
	}

	if (intel_gen(devid) >= 8) {
		uint32_t slice_freq0, slice_freq1, unslice_freq0, unslice_freq1;
		const char *reason0 = gen8_read_report_reason(oa_report0);
		const char *reason1 = gen8_read_report_reason(oa_report1);

		gen8_read_report_clock_ratios(oa_report0,
					      &slice_freq0, &unslice_freq0);
		gen8_read_report_clock_ratios(oa_report1,
					      &slice_freq1, &unslice_freq1);

		igt_debug("SLICE CLK: 1st = %umhz, 2nd = %umhz, delta = %d\n",
			  slice_freq0, slice_freq1,
			  ((int)slice_freq1 - (int)slice_freq0));
		igt_debug("UNSLICE CLK: 1st = %umhz, 2nd = %umhz, delta = %d\n",
			  unslice_freq0, unslice_freq1,
			  ((int)unslice_freq1 - (int)unslice_freq0));

		igt_debug("REASONS: 1st = \"%s\", 2nd = \"%s\"\n", reason0, reason1);
	}

	/* Gen8+ has some 40bit A counters... */
	for (int j = 0; j < oa_formats[fmt].n_a40; j++) {
		uint64_t value0 = gen8_read_40bit_a_counter(oa_report0, fmt, j);
		uint64_t value1 = gen8_read_40bit_a_counter(oa_report1, fmt, j);
		uint64_t delta = gen8_40bit_a_delta(value0, value1);

		if (undefined_a_counters[j])
			continue;

		igt_debug("A%d: 1st = %"PRIu64", 2nd = %"PRIu64", delta = %"PRIu64"\n",
			  j, value0, value1, delta);
	}

	for (int j = 0; j < oa_formats[fmt].n_a; j++) {
		uint32_t *a0 = (uint32_t *)(((uint8_t *)oa_report0) +
					    oa_formats[fmt].a_off);
		uint32_t *a1 = (uint32_t *)(((uint8_t *)oa_report1) +
					    oa_formats[fmt].a_off);
		int a_id = oa_formats[fmt].first_a + j;
		uint32_t delta = a1[j] - a0[j];

		if (undefined_a_counters[a_id])
			continue;

		igt_debug("A%d: 1st = %"PRIu32", 2nd = %"PRIu32", delta = %"PRIu32"\n",
			  a_id, a0[j], a1[j], delta);
	}

	for (int j = 0; j < oa_formats[fmt].n_b; j++) {
		uint32_t *b0 = (uint32_t *)(((uint8_t *)oa_report0) +
					    oa_formats[fmt].b_off);
		uint32_t *b1 = (uint32_t *)(((uint8_t *)oa_report1) +
					    oa_formats[fmt].b_off);
		uint32_t delta = b1[j] - b0[j];

		igt_debug("B%d: 1st = %"PRIu32", 2nd = %"PRIu32", delta = %"PRIu32"\n",
			  j, b0[j], b1[j], delta);
	}

	for (int j = 0; j < oa_formats[fmt].n_c; j++) {
		uint32_t *c0 = (uint32_t *)(((uint8_t *)oa_report0) +
					    oa_formats[fmt].c_off);
		uint32_t *c1 = (uint32_t *)(((uint8_t *)oa_report1) +
					    oa_formats[fmt].c_off);
		uint32_t delta = c1[j] - c0[j];

		igt_debug("C%d: 1st = %"PRIu32", 2nd = %"PRIu32", delta = %"PRIu32"\n",
			  j, c0[j], c1[j], delta);
	}
}

static void
print_report(uint32_t *report, int fmt)
{
	igt_debug("TIMESTAMP: %"PRIu32"\n", report[1]);

	if (IS_HASWELL(devid) && oa_formats[fmt].n_c == 0) {
		igt_debug("CLOCK = N/A\n");
	} else {
		uint32_t clock = read_report_ticks(report, fmt);

		igt_debug("CLOCK: %"PRIu32"\n", clock);
	}

	if (intel_gen(devid) >= 8) {
		uint32_t slice_freq, unslice_freq;
		const char *reason = gen8_read_report_reason(report);

		gen8_read_report_clock_ratios(report, &slice_freq, &unslice_freq);

		igt_debug("SLICE CLK: %umhz\n", slice_freq);
		igt_debug("UNSLICE CLK: %umhz\n", unslice_freq);
		igt_debug("REASON: \"%s\"\n", reason);
		igt_debug("CTX ID: %"PRIu32"/%"PRIx32"\n", report[2], report[2]);
	}

	/* Gen8+ has some 40bit A counters... */
	for (int j = 0; j < oa_formats[fmt].n_a40; j++) {
		uint64_t value = gen8_read_40bit_a_counter(report, fmt, j);

		if (undefined_a_counters[j])
			continue;

		igt_debug("A%d: %"PRIu64"\n", j, value);
	}

	for (int j = 0; j < oa_formats[fmt].n_a; j++) {
		uint32_t *a = (uint32_t *)(((uint8_t *)report) +
					   oa_formats[fmt].a_off);
		int a_id = oa_formats[fmt].first_a + j;

		if (undefined_a_counters[a_id])
			continue;

		igt_debug("A%d: %"PRIu32"\n", a_id, a[j]);
	}

	for (int j = 0; j < oa_formats[fmt].n_b; j++) {
		uint32_t *b = (uint32_t *)(((uint8_t *)report) +
					   oa_formats[fmt].b_off);

		igt_debug("B%d: %"PRIu32"\n", j, b[j]);
	}

	for (int j = 0; j < oa_formats[fmt].n_c; j++) {
		uint32_t *c = (uint32_t *)(((uint8_t *)report) +
					   oa_formats[fmt].c_off);

		igt_debug("C%d: %"PRIu32"\n", j, c[j]);
	}
}

static void
test_oa_formats(void)
{
	for (int i = 0; i < ARRAY_SIZE(oa_formats); i++) {
		uint32_t oa_report0[64];
		uint32_t oa_report1[64];

		if (!oa_formats[i].name) /* sparse, indexed by ID */
			continue;

		if (oa_formats[i].min_gen &&
		    intel_gen(devid) < oa_formats[i].min_gen) {
			igt_debug("skipping unsupported OA format %s\n",
				  oa_formats[i].name);
			continue;
		}

		if (oa_formats[i].max_gen &&
		    intel_gen(devid) > oa_formats[i].max_gen) {
			igt_debug("skipping unsupported OA format %s\n",
				  oa_formats[i].name);
			continue;
		}

		igt_debug("Checking OA format %s\n", oa_formats[i].name);

		open_and_read_2_oa_reports(i,
					   oa_exp_1_millisec,
					   oa_report0,
					   oa_report1,
					   false); /* timer reports only */

		print_reports(oa_report0, oa_report1, i);
		sanity_check_reports(oa_report0, oa_report1, i);
	}
}


enum load {
	LOW,
	HIGH
};

static struct load_helper {
	int devid;
	int has_ppgtt;
	drm_intel_bufmgr *bufmgr;
	struct intel_batchbuffer *batch;
	drm_intel_bo *target_buffer;
	enum load load;
	bool exit;
	struct igt_helper_process igt_proc;
	drm_intel_bo *src, *dst;
} lh;

static void load_helper_signal_handler(int sig)
{
	if (sig == SIGUSR2)
		lh.load = lh.load == LOW ? HIGH : LOW;
	else
		lh.exit = true;
}

static void emit_store_dword_imm(uint32_t val)
{
	int cmd;
	struct intel_batchbuffer *batch = lh.batch;

	cmd = MI_STORE_DWORD_IMM;
	if (!lh.has_ppgtt)
		cmd |= MI_MEM_VIRTUAL;

	BEGIN_BATCH(4, 0); /* just ignore the reloc we emit and count dwords */
	OUT_BATCH(cmd);
	if (batch->gen >= 8) {
		OUT_RELOC(lh.target_buffer, I915_GEM_DOMAIN_INSTRUCTION,
			  I915_GEM_DOMAIN_INSTRUCTION, 0);
	} else {
		OUT_BATCH(0); /* reserved */
		OUT_RELOC(lh.target_buffer, I915_GEM_DOMAIN_INSTRUCTION,
			  I915_GEM_DOMAIN_INSTRUCTION, 0);
	}
	OUT_BATCH(val);
	ADVANCE_BATCH();
}

#define LOAD_HELPER_PAUSE_USEC 500
#define LOAD_HELPER_BO_SIZE (16*1024*1024)
static void load_helper_set_load(enum load load)
{
	igt_assert(lh.igt_proc.running);

	if (lh.load == load)
		return;

	lh.load = load;
	kill(lh.igt_proc.pid, SIGUSR2);
}

static void load_helper_run(enum load load)
{
	/*
	 * FIXME fork helpers won't get cleaned up when started from within a
	 * subtest, so handle the case where it sticks around a bit too long.
	 */
	if (lh.igt_proc.running) {
		load_helper_set_load(load);
		return;
	}

	lh.load = load;

	igt_fork_helper(&lh.igt_proc) {
		const uint32_t bbe = MI_BATCH_BUFFER_END;
		struct drm_i915_gem_exec_object2 object;
		struct drm_i915_gem_execbuffer2 execbuf;
		uint32_t fences[3];
		uint32_t val = 0;

		signal(SIGUSR1, load_helper_signal_handler);
		signal(SIGUSR2, load_helper_signal_handler);

		fences[0] = gem_create(drm_fd, 4096);
		gem_write(drm_fd, fences[0], 0,  &bbe, sizeof(bbe));
		fences[1] = gem_create(drm_fd, 4096);
		gem_write(drm_fd, fences[1], 0,  &bbe, sizeof(bbe));
		fences[2] = gem_create(drm_fd, 4096);
		gem_write(drm_fd, fences[2], 0,  &bbe, sizeof(bbe));

		memset(&execbuf, 0, sizeof(execbuf));
		execbuf.buffers_ptr = (uintptr_t)&object;
		execbuf.buffer_count = 1;
		if (intel_gen(lh.devid) >= 6)
			execbuf.flags = I915_EXEC_BLT;

		while (!lh.exit) {
			memset(&object, 0, sizeof(object));
			object.handle = fences[val%3];

			while (gem_bo_busy(drm_fd, object.handle))
				usleep(100);

			if (lh.load == HIGH)
				intel_copy_bo(lh.batch, lh.dst, lh.src,
					      LOAD_HELPER_BO_SIZE);

			emit_store_dword_imm(val);
			intel_batchbuffer_flush_on_ring(lh.batch,
                                                        I915_EXEC_BLT);
			val++;

			gem_execbuf(drm_fd, &execbuf);

			/* Lower the load by pausing after every submitted
			 * write. */
			if (lh.load == LOW)
				usleep(LOAD_HELPER_PAUSE_USEC);
		}

		/* Wait for completion without boosting */
		usleep(1000);
		while (gem_bo_busy(drm_fd, lh.target_buffer->handle))
			usleep(1000);

		igt_debug("load helper sent %u dword writes\n", val);
		gem_close(drm_fd, fences[0]);
		gem_close(drm_fd, fences[1]);
		gem_close(drm_fd, fences[2]);
	}
}

static void load_helper_stop(void)
{
	kill(lh.igt_proc.pid, SIGUSR1);
	igt_assert(igt_wait_helper(&lh.igt_proc) == 0);
}

static void load_helper_init(void)
{
	lh.devid = intel_get_drm_devid(drm_fd);
	lh.has_ppgtt = gem_uses_ppgtt(drm_fd);

	/* MI_STORE_DATA can only use GTT address on gen4+/g33 and needs
	 * snoopable mem on pre-gen6. Hence load-helper only works on gen6+, but
	 * that's also all we care about for the rps testcase*/
	igt_assert(intel_gen(lh.devid) >= 6);
	lh.bufmgr = drm_intel_bufmgr_gem_init(drm_fd, 4096);
	igt_assert(lh.bufmgr);

	drm_intel_bufmgr_gem_enable_reuse(lh.bufmgr);

	lh.batch = intel_batchbuffer_alloc(lh.bufmgr, lh.devid);
	igt_assert(lh.batch);

	lh.target_buffer = drm_intel_bo_alloc(lh.bufmgr, "target bo",
					      4096, 4096);
	igt_assert(lh.target_buffer);

	lh.dst = drm_intel_bo_alloc(lh.bufmgr, "dst bo",
				    LOAD_HELPER_BO_SIZE, 4096);
	igt_assert(lh.dst);
	lh.src = drm_intel_bo_alloc(lh.bufmgr, "src bo",
				    LOAD_HELPER_BO_SIZE, 4096);
	igt_assert(lh.src);
}

static void load_helper_deinit(void)
{
	if (lh.igt_proc.running)
		load_helper_stop();

	if (lh.target_buffer)
		drm_intel_bo_unreference(lh.target_buffer);
	if (lh.src)
		drm_intel_bo_unreference(lh.src);
	if (lh.dst)
		drm_intel_bo_unreference(lh.dst);

	if (lh.batch)
		intel_batchbuffer_free(lh.batch);

	if (lh.bufmgr)
		drm_intel_bufmgr_destroy(lh.bufmgr);
}

static void
test_oa_exponents(int gt_freq_mhz)
{
	uint32_t freq_margin;

	/* This test tries to use the sysfs interface for pinning the GT
	 * frequency so we have another point of reference for comparing with
	 * the clock frequency as derived from OA reports.
	 *
	 * This test has been finicky to stabilise while the
	 * gt_min/max_freq_mhz files in sysfs don't seem to be a reliable
	 * mechanism for fixing the gpu frequency.
	 *
	 * Since these unit tests are focused on the OA unit not the ability to
	 * pin the frequency via sysfs we make the test account for pinning not
	 * being reliable and read back the current frequency for each
	 * iteration of this test to take this into account.
	 */
	gt_frequency_pin(gt_freq_mhz);

	load_helper_init();
	load_helper_run(LOW);

	igt_debug("Testing OA timer exponents with requested GT frequency = %dmhz\n",
		  gt_freq_mhz);

	/* allow a +- 10% error margin when checking that the frequency
	 * calculated from the OA reports matches the frequency according to
	 * sysfs.
	 */
	freq_margin = gt_freq_mhz * 0.1;

	/* It's asking a lot to sample with a 160 nanosecond period and the
	 * test can fail due to buffer overflows if it wasn't possible to
	 * keep up, so we don't start from an exponent of zero...
	 */
	for (int i = 5; i < 20; i++) {
		uint32_t expected_timestamp_delta;
		uint32_t timestamp_delta;
		uint32_t oa_report0[64];
		uint32_t oa_report1[64];
		uint32_t time_delta;
		uint32_t clock_delta;
		uint32_t freq;
		int n_tested = 0;
		int n_freq_matches = 0;
		int n_time_delta_matches = 0;

#warning "XXX: it seems pretty odd that the time delta assertion failures centre around these exponents"
		if (i == 6 || i == 7 || i == 8)
			continue;

		/* The exponent is effectively selecting a bit in the timestamp
		 * to trigger reports on and so in practice we expect the raw
		 * timestamp deltas for periodic reports to exactly match the
		 * value of next bit.
		 */
		expected_timestamp_delta = 2 << i;

		for (int j = 0; n_tested < 10 && j < 100; j++) {
			int gt_freq_mhz_0, gt_freq_mhz_1;
			uint32_t ticks0, ticks1;

			gt_freq_mhz_0 = sysfs_read("gt_act_freq_mhz");

			igt_debug("ITER %d: testing OA exponent %d (period = %"PRIu64"ns) with sysfs GT freq = %dmhz +- %u\n",
				  j, i,
				  oa_exponent_to_ns(i),
				  gt_freq_mhz_0, freq_margin);

			open_and_read_2_oa_reports(test_oa_format,
						   i, /* exponent */
						   oa_report0,
						   oa_report1,
						   true); /* timer triggered
							     reports only */

			gt_freq_mhz_1 = sysfs_read("gt_act_freq_mhz");

			/* If it looks like the frequency has changed according
			 * to sysfs then skip looking at this pair of reports
			 */
			if (gt_freq_mhz_0 != gt_freq_mhz_1) {
				igt_debug("skipping OA reports pair due to GT"
					  " frequency change according to sysfs %i/%i\n",
					  gt_freq_mhz_0, gt_freq_mhz_1);
				continue;
			}

			timestamp_delta = oa_report1[1] - oa_report0[1];
			igt_assert_neq(timestamp_delta, 0);

			time_delta = timebase_scale(timestamp_delta);

			ticks0 = read_report_ticks(oa_report0, test_oa_format);
			ticks1 = read_report_ticks(oa_report1, test_oa_format);
			clock_delta = ticks1 - ticks0;

			freq = ((uint64_t)clock_delta * 1000) / time_delta;
			igt_debug("ITER %d: time delta = %"PRIu32"(ns) clock delta = %"PRIu32" freq = %"PRIu32"(mhz)\n",
				  j, time_delta, clock_delta, freq);

			igt_debug("est_timestamp=%u\n", gt_freq_mhz_0 * clock_delta);


			if (timestamp_delta == expected_timestamp_delta)
				n_time_delta_matches++;
			else {
				igt_debug("timestamp delta mismatch: %"PRIu64"ns != expected %"PRIu64"ns, ts0 = %u/0x%x, ts1 = %u/0x%x\n",
					  timebase_scale(timestamp_delta),
					  timebase_scale(expected_timestamp_delta),
					  oa_report0[1], oa_report0[1],
					  oa_report1[1], oa_report1[1]);
				print_reports(oa_report0, oa_report1, test_oa_format);
				igt_assert(timestamp_delta <
					   (expected_timestamp_delta * 2));
			}

                        if (freq < (gt_freq_mhz_1 + freq_margin) &&
                            freq > (gt_freq_mhz_1 - freq_margin))
				n_freq_matches++;
			else {
				igt_debug("unexpected frequency %i (expected range %i-%i)\n",
					  freq, gt_freq_mhz_1 - freq_margin,
					  gt_freq_mhz_1 + freq_margin);
			}

			n_tested++;
		}

		if (n_tested < 10)
			igt_debug("sysfs frequency pinning too unstable for cross-referencing with OA derived frequency");
		igt_assert_eq(n_tested, 10);

		igt_debug("number of iterations with expected timestamp delta = %d\n",
			  n_time_delta_matches);

		/* The HW doesn't give us any strict guarantee that the
		 * timestamps are exactly aligned with the exponent mask but
		 * in practice it seems very rare for that not to be the case
		 * so it a useful sanity check to assert quite strictly...
		 */
		igt_assert(n_time_delta_matches >= 9);

		igt_debug("number of iterations with expected clock frequency = %d\n",
			  n_freq_matches);

		/* Don't assert the calculated frequency for extremely short
		 * durations.
		 *
		 * Allow some mismatches since can't be can't be sure about
		 * frequency changes between sysfs reads.
		 */
		if (i > 3)
			igt_assert(n_freq_matches >= 7);
	}

	gt_frequency_range_restore();

	load_helper_stop();
	load_helper_deinit();
}

/* The OA exponent selects a timestamp counter bit to trigger reports on.
 *
 * With a 64bit timestamp and least significant bit approx == 80ns then the MSB
 * equates to > 40 thousand years and isn't exposed via the i915 perf interface.
 *
 * The max exponent exposed is expected to be 31, which is still a fairly
 * ridiculous period (>5min) but is the maximum exponent where it's still
 * possible to use periodic sampling as a means for tracking the overflow of
 * 32bit OA report timestamps.
 */
static void
test_invalid_oa_exponent(void)
{
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, 31, /* maximum exponent expected
						       to be accepted */
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};

	stream_fd = __perf_open(drm_fd, &param);

	__perf_close(stream_fd);

	for (int i = 32; i < 65; i++) {
		properties[7] = i;
		do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EINVAL);
	}
}

/* The lowest periodic sampling exponent equates to a period of 160 nanoseconds
 * or a frequency of 6.25MHz which is only possible to request as root by
 * default. By default the maximum OA sampling rate is 100KHz
 */
static void
test_low_oa_exponent_permissions(void)
{
	int max_freq = read_u64_file("/proc/sys/dev/i915/oa_max_sample_rate");
	int bad_exponent = max_oa_exponent_for_freq_gt(max_freq);
	int ok_exponent = bad_exponent + 1;
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, bad_exponent,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	uint64_t oa_period, oa_freq;

	igt_assert_eq(max_freq, 100000);

	/* Avoid EACCES errors opening a stream without CAP_SYS_ADMIN */
	write_u64_file("/proc/sys/dev/i915/perf_stream_paranoid", 0);

	igt_fork(child, 1) {
		igt_drop_root();

		do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EACCES);
	}

	igt_waitchildren();

	properties[7] = ok_exponent;

	igt_fork(child, 1) {
		igt_drop_root();

		stream_fd = __perf_open(drm_fd, &param);
		__perf_close(stream_fd);
	}

	igt_waitchildren();

	oa_period = timebase_scale(2 << ok_exponent);
	oa_freq = NSEC_PER_SEC / oa_period;
	write_u64_file("/proc/sys/dev/i915/oa_max_sample_rate", oa_freq - 100);

	igt_fork(child, 1) {
		igt_drop_root();

		do_ioctl_err(drm_fd, DRM_IOCTL_I915_PERF_OPEN, &param, EACCES);
	}

	igt_waitchildren();

	/* restore the defaults */
	write_u64_file("/proc/sys/dev/i915/oa_max_sample_rate", 100000);
	write_u64_file("/proc/sys/dev/i915/perf_stream_paranoid", 1);
}

static void
test_per_context_mode_unprivileged(void)
{
	uint64_t properties[] = {
		/* Single context sampling */
		DRM_I915_PERF_PROP_CTX_HANDLE, UINT64_MAX, /* updated below */

		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exp_1_millisec,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};

	/* should be default, but just to be sure... */
	write_u64_file("/proc/sys/dev/i915/perf_stream_paranoid", 1);

	igt_fork(child, 1) {
		drm_intel_context *context;
		drm_intel_bufmgr *bufmgr;
		uint32_t ctx_id = 0xffffffff; /* invalid id */
		int ret;

		igt_drop_root();

		bufmgr = drm_intel_bufmgr_gem_init(drm_fd, 4096);
		context = drm_intel_gem_context_create(bufmgr);

		igt_assert(context);

		ret = drm_intel_gem_context_get_id(context, &ctx_id);
		igt_assert_eq(ret, 0);
		igt_assert_neq(ctx_id, 0xffffffff);

		properties[1] = ctx_id;

		stream_fd = __perf_open(drm_fd, &param);
		__perf_close(stream_fd);

		drm_intel_gem_context_destroy(context);
		drm_intel_bufmgr_destroy(bufmgr);
	}

	igt_waitchildren();
}

static int64_t
get_time(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);

	return ts.tv_sec * 1000000000 + ts.tv_nsec;
}

/* Note: The interface doesn't currently provide strict guarantees or control
 * over the upper bound for how long it might take for a POLLIN event after
 * some OA report is written by the OA unit.
 *
 * The plan is to add a property later that gives some control over the maximum
 * latency, but for now we expect it is tuned for a fairly low latency
 * suitable for applications wanting to provide live feedback for captured
 * metrics.
 *
 * At the time of writing this test the driver was using a fixed 200Hz hrtimer
 * regardless of the OA sampling exponent.
 *
 * There is no lower bound since a stream configured for periodic sampling may
 * still contain other automatically triggered reports.
 *
 * What we try and check for here is that blocking reads don't return EAGAIN
 * and that we aren't spending any significant time burning the cpu in
 * kernelspace.
 */
static void
test_blocking(void)
{
	/* ~40 milliseconds
	 *
	 * Having a period somewhat > sysconf(_SC_CLK_TCK) helps to stop
	 * scheduling (liable to kick in when we make blocking poll()s/reads)
	 * from interfering with the test.
	 */
	int oa_exponent = max_oa_exponent_for_period_lte(40000000);
	uint64_t oa_period = oa_exponent_to_ns(oa_exponent);
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exponent,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	uint8_t buf[1024 * 1024];
	struct tms start_times;
	struct tms end_times;
	int64_t user_ns, kernel_ns;
	int64_t tick_ns = 1000000000 / sysconf(_SC_CLK_TCK);
	int64_t test_duration_ns = tick_ns * 1000;

	int max_iterations = (test_duration_ns / oa_period) + 1;
	int n_extra_iterations = 0;

	/* It's a bit tricky to put a lower limit here, but we expect a
	 * relatively low latency for seeing reports, while we don't currently
	 * give any control over this in the api.
	 *
	 * We assume a maximum latency of 6 millisecond to deliver a POLLIN and
	 * read() after a new sample is written (46ms per iteration) considering
	 * the knowledge that that the driver uses a 200Hz hrtimer (5ms period)
	 * to check for data and giving some time to read().
	 */
	int min_iterations = (test_duration_ns / (oa_period + 6000000ull));

	int64_t start;
	int n = 0;

	stream_fd = __perf_open(drm_fd, &param);

	times(&start_times);

	igt_debug("tick length = %dns, test duration = %"PRIu64"ns, min iter. = %d, max iter. = %d\n",
		  (int)tick_ns, test_duration_ns,
		  min_iterations, max_iterations);

	/* In the loop we perform blocking polls while the HW is sampling at
	 * ~25Hz, with the expectation that we spend most of our time blocked
	 * in the kernel, and shouldn't be burning cpu cycles in the kernel in
	 * association with this process (verified by looking at stime before
	 * and after loop).
	 *
	 * We're looking to assert that less than 1% of the test duration is
	 * spent in the kernel dealing with polling and read()ing.
	 *
	 * The test runs for a relatively long time considering the very low
	 * resolution of stime in ticks of typically 10 milliseconds. Since we
	 * don't know the fractional part of tick values we read from userspace
	 * so our minimum threshold needs to be >= one tick since any
	 * measurement might really be +- tick_ns (assuming we effectively get
	 * floor(real_stime)).
	 *
	 * We Loop for 1000 x tick_ns so one tick corresponds to 0.1%
	 */
	for (start = get_time(); (get_time() - start) < test_duration_ns; /* nop */) {
		struct drm_i915_perf_record_header *header;
		bool timer_report_read = false;
		bool non_timer_report_read = false;
		int ret;

		while ((ret = read(stream_fd, buf, sizeof(buf))) < 0 &&
		       errno == EINTR)
			;

		igt_assert(ret > 0);

		/* For Haswell reports don't contain a well defined reason
		 * field we so assume all reports to be 'periodic'. For gen8+
		 * we want to to consider that the HW automatically writes some
		 * non periodic reports (e.g. on context switch) which might
		 * lead to more successful read()s than expected due to
		 * periodic sampling and we don't want these extra reads to
		 * cause the test to fail...
		 */
		if (intel_gen(devid) >= 8) {
			for (int offset = 0; offset < ret; offset += header->size) {
				header = (void *)(buf + offset);

				if (header->type == DRM_I915_PERF_RECORD_SAMPLE) {
					uint32_t *report = (void *)(header + 1);

					uint32_t reason = ((report[0] >>
							    OAREPORT_REASON_SHIFT) &
							   OAREPORT_REASON_MASK);

					if (reason & OAREPORT_REASON_TIMER)
						timer_report_read = true;
					else
						non_timer_report_read = true;
				}
			}
		}

		if (non_timer_report_read && !timer_report_read)
			n_extra_iterations++;

		n++;
	}

	times(&end_times);

	/* Using nanosecond units is fairly silly here, given the tick in-
	 * precision - ah well, it's consistent with the get_time() units.
	 */
	user_ns = (end_times.tms_utime - start_times.tms_utime) * tick_ns;
	kernel_ns = (end_times.tms_stime - start_times.tms_stime) * tick_ns;

	igt_debug("%d blocking reads during test with ~25Hz OA sampling (expect no more than %d)\n",
		  n, max_iterations);
	igt_debug("%d extra iterations seen, not related to periodic sampling (e.g. context switches)\n",
		  n_extra_iterations);
	igt_debug("time in userspace = %"PRIu64"ns (+-%dns) (start utime = %d, end = %d)\n",
		  user_ns, (int)tick_ns,
		  (int)start_times.tms_utime, (int)end_times.tms_utime);
	igt_debug("time in kernelspace = %"PRIu64"ns (+-%dns) (start stime = %d, end = %d)\n",
		  kernel_ns, (int)tick_ns,
		  (int)start_times.tms_stime, (int)end_times.tms_stime);

	/* Technically, more of a HW sanity check but e.g. on SKL by default
	 * slice/unslice clock ratio reports can drown out periodic reports.
	 *
	 * We want to notice if there are (non-periodic) automatic reports
	 * being generated at an unexpectedly high frequency since they
	 * will probably cause excessive work in userspace to process.
	 */
	igt_assert(n_extra_iterations < (max_iterations / 2));

	/* With completely broken blocking (but also not returning an error) we
	 * could end up with an open loop,
	 */
	igt_assert(n <= (max_iterations + n_extra_iterations));

	/* Make sure the driver is reporting new samples with a reasonably
	 * low latency...
	 */
	igt_assert(n > (min_iterations + n_extra_iterations));

	igt_assert(kernel_ns <= (test_duration_ns / 100ull));

	__perf_close(stream_fd);
}

static void
test_polling(void)
{
	/* ~40 milliseconds
	 *
	 * Having a period somewhat > sysconf(_SC_CLK_TCK) helps to stop
	 * scheduling (liable to kick in when we make blocking poll()s/reads)
	 * from interfering with the test.
	 */
	int oa_exponent = max_oa_exponent_for_period_lte(40000000);
	uint64_t oa_period = oa_exponent_to_ns(oa_exponent);
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exponent,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC |
			I915_PERF_FLAG_FD_NONBLOCK,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	uint8_t buf[1024 * 1024];
	struct tms start_times;
	struct tms end_times;
	int64_t user_ns, kernel_ns;
	int64_t tick_ns = 1000000000 / sysconf(_SC_CLK_TCK);
	int64_t test_duration_ns = tick_ns * 1000;

	int max_iterations = (test_duration_ns / oa_period) + 1;
	int n_extra_iterations = 0;

	/* It's a bit tricky to put a lower limit here, but we expect a
	 * relatively low latency for seeing reports, while we don't currently
	 * give any control over this in the api.
	 *
	 * We assume a maximum latency of 6 millisecond to deliver a POLLIN and
	 * read() after a new sample is written (46ms per iteration) considering
	 * the knowledge that that the driver uses a 200Hz hrtimer (5ms period)
	 * to check for data and giving some time to read().
	 */
	int min_iterations = (test_duration_ns / (oa_period + 6000000ull));
	int64_t start;
	int n = 0;

	stream_fd = __perf_open(drm_fd, &param);

	times(&start_times);

	igt_debug("tick length = %dns, test duration = %"PRIu64"ns, min iter. = %d, max iter. = %d\n",
		  (int)tick_ns, test_duration_ns,
		  min_iterations, max_iterations);

	/* In the loop we perform blocking polls while the HW is sampling at
	 * ~25Hz, with the expectation that we spend most of our time blocked
	 * in the kernel, and shouldn't be burning cpu cycles in the kernel in
	 * association with this process (verified by looking at stime before
	 * and after loop).
	 *
	 * We're looking to assert that less than 1% of the test duration is
	 * spent in the kernel dealing with polling and read()ing.
	 *
	 * The test runs for a relatively long time considering the very low
	 * resolution of stime in ticks of typically 10 milliseconds. Since we
	 * don't know the fractional part of tick values we read from userspace
	 * so our minimum threshold needs to be >= one tick since any
	 * measurement might really be +- tick_ns (assuming we effectively get
	 * floor(real_stime)).
	 *
	 * We Loop for 1000 x tick_ns so one tick corresponds to 0.1%
	 */
	for (start = get_time(); (get_time() - start) < test_duration_ns; /* nop */) {
		struct pollfd pollfd = { .fd = stream_fd, .events = POLLIN };
		struct drm_i915_perf_record_header *header;
		bool timer_report_read = false;
		bool non_timer_report_read = false;
		int ret;

		while ((ret = poll(&pollfd, 1, -1)) < 0 &&
		       errno == EINTR)
			;
		igt_assert_eq(ret, 1);
		igt_assert(pollfd.revents & POLLIN);

		while ((ret = read(stream_fd, buf, sizeof(buf))) < 0 &&
		       errno == EINTR)
			;

		/* Don't expect to see EAGAIN if we've had a POLLIN event
		 *
		 * XXX: actually this is technically overly strict since we do
		 * knowingly allow false positive POLLIN events. At least in
		 * the future when supporting context filtering of metrics for
		 * Gen8+ handled in the kernel then POLLIN events may be
		 * delivered when we know there are pending reports to process
		 * but before we've done any filtering to know for certain that
		 * any reports are destined to be copied to userspace.
		 *
		 * Still, for now it's a reasonable sanity check.
		 */
		if (ret < 0)
			igt_debug("Unexpected error when reading after poll = %d\n", errno);
		igt_assert_neq(ret, -1);

		/* For Haswell reports don't contain a well defined reason
		 * field we so assume all reports to be 'periodic'. For gen8+
		 * we want to to consider that the HW automatically writes some
		 * non periodic reports (e.g. on context switch) which might
		 * lead to more successful read()s than expected due to
		 * periodic sampling and we don't want these extra reads to
		 * cause the test to fail...
		 */
		if (intel_gen(devid) >= 8) {
			for (int offset = 0; offset < ret; offset += header->size) {
				header = (void *)(buf + offset);

				if (header->type == DRM_I915_PERF_RECORD_SAMPLE) {
					uint32_t *report = (void *)(header + 1);

					uint32_t reason = ((report[0] >>
							    OAREPORT_REASON_SHIFT) &
							   OAREPORT_REASON_MASK);

					if (reason & OAREPORT_REASON_TIMER)
						timer_report_read = true;
					else
						non_timer_report_read = true;
				}
			}
		}

		if (non_timer_report_read && !timer_report_read)
			n_extra_iterations++;

		/* At this point, after consuming pending reports (and hoping
		 * the scheduler hasn't stopped us for too long we now
		 * expect EAGAIN on read.
		 */
		while ((ret = read(stream_fd, buf, sizeof(buf))) < 0 &&
		       errno == EINTR)
			;
		igt_assert_eq(ret, -1);
		igt_assert_eq(errno, EAGAIN);

		n++;
	}

	times(&end_times);

	/* Using nanosecond units is fairly silly here, given the tick in-
	 * precision - ah well, it's consistent with the get_time() units.
	 */
	user_ns = (end_times.tms_utime - start_times.tms_utime) * tick_ns;
	kernel_ns = (end_times.tms_stime - start_times.tms_stime) * tick_ns;

	igt_debug("%d blocking reads during test with ~25Hz OA sampling (expect no more than %d)\n",
		  n, max_iterations);
	igt_debug("%d extra iterations seen, not related to periodic sampling (e.g. context switches)\n",
		  n_extra_iterations);
	igt_debug("time in userspace = %"PRIu64"ns (+-%dns) (start utime = %d, end = %d)\n",
		  user_ns, (int)tick_ns,
		  (int)start_times.tms_utime, (int)end_times.tms_utime);
	igt_debug("time in kernelspace = %"PRIu64"ns (+-%dns) (start stime = %d, end = %d)\n",
		  kernel_ns, (int)tick_ns,
		  (int)start_times.tms_stime, (int)end_times.tms_stime);

	/* Technically, more of a HW sanity check but e.g. on SKL by default
	 * slice/unslice clock ratio reports can drown out periodic reports.
	 *
	 * We want to notice if there are (non-periodic) automatic reports
	 * being generated at an unexpectedly high frequency since they
	 * will probably cause excessive work in userspace to process.
	 */
	igt_assert(n_extra_iterations < (max_iterations / 2));

	/* With completely broken blocking while polling (but still somehow
	 * reporting a POLLIN event) we could end up with an open loop.
	 */
	igt_assert(n <= (max_iterations + n_extra_iterations));

	/* Make sure the driver is reporting new samples with a reasonably
	 * low latency...
	 */
	igt_assert(n > (min_iterations + n_extra_iterations));

	igt_assert(kernel_ns <= (test_duration_ns / 100ull));

	__perf_close(stream_fd);
}

static void
test_buffer_fill(void)
{
	/* ~5 micro second period */
	int oa_exponent = max_oa_exponent_for_period_lte(5000);
	uint64_t oa_period = oa_exponent_to_ns(oa_exponent);
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exponent,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	int buf_size = 65536 * (256 + sizeof(struct drm_i915_perf_record_header));
	uint8_t *buf = malloc(buf_size);
	size_t oa_buf_size = 16 * 1024 * 1024;
	size_t report_size = oa_formats[test_oa_format].size;
	int n_full_oa_reports = oa_buf_size / report_size;
	uint64_t fill_duration = n_full_oa_reports * oa_period;

	igt_assert(fill_duration < 1000000000);

	stream_fd = __perf_open(drm_fd, &param);

	for (int i = 0; i < 5; i++) {
		struct drm_i915_perf_record_header *header;
		bool overflow_seen;
		int offset = 0;
		int len;

		nanosleep(&(struct timespec){ .tv_sec = 0,
					      .tv_nsec = fill_duration * 1.25 },
			  NULL);

		while ((len = read(stream_fd, buf, buf_size)) == -1 && errno == EINTR)
			;

		igt_assert_neq(len, -1);

		overflow_seen = false;
		for (offset = 0; offset < len; offset += header->size) {
			header = (void *)(buf + offset);

			if (header->type == DRM_I915_PERF_RECORD_OA_BUFFER_LOST)
				overflow_seen = true;
		}

		igt_assert_eq(overflow_seen, true);

		nanosleep(&(struct timespec){ .tv_sec = 0,
					      .tv_nsec = fill_duration / 2 },
			  NULL);

		while ((len = read(stream_fd, buf, buf_size)) == -1 && errno == EINTR)
			;

		igt_assert_neq(len, -1);

		igt_assert(len > report_size * n_full_oa_reports * 0.45);
		igt_assert(len < report_size * n_full_oa_reports * 0.55);

		overflow_seen = false;
		for (offset = 0; offset < len; offset += header->size) {
			header = (void *)(buf + offset);

			if (header->type == DRM_I915_PERF_RECORD_OA_BUFFER_LOST)
				overflow_seen = true;
		}

		igt_assert_eq(overflow_seen, false);
	}

	free(buf);

	__perf_close(stream_fd);
}

static void
test_enable_disable(void)
{
	/* ~5 micro second period */
	int oa_exponent = max_oa_exponent_for_period_lte(5000);
	uint64_t oa_period = oa_exponent_to_ns(oa_exponent);
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exponent,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC |
			 I915_PERF_FLAG_DISABLED, /* Verify we start disabled */
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	int buf_size = 65536 * (256 + sizeof(struct drm_i915_perf_record_header));
	uint8_t *buf = malloc(buf_size);
	size_t oa_buf_size = 16 * 1024 * 1024;
	size_t report_size = oa_formats[test_oa_format].size;
	int n_full_oa_reports = oa_buf_size / report_size;
	uint64_t fill_duration = n_full_oa_reports * oa_period;

	stream_fd = __perf_open(drm_fd, &param);

	for (int i = 0; i < 5; i++) {
		int len;

		/* Giving enough time for an overflow might help catch whether
		 * the OA unit has been enabled even if the driver might at
		 * least avoid copying reports while disabled.
		 */
		nanosleep(&(struct timespec){ .tv_sec = 0,
					      .tv_nsec = fill_duration * 1.25 },
			  NULL);

		while ((len = read(stream_fd, buf, buf_size)) == -1 && errno == EINTR)
			;

		igt_assert_eq(len, -1);
		igt_assert_eq(errno, EIO);

		do_ioctl(stream_fd, I915_PERF_IOCTL_ENABLE, 0);

		nanosleep(&(struct timespec){ .tv_sec = 0,
					      .tv_nsec = fill_duration / 2 },
			  NULL);

		while ((len = read(stream_fd, buf, buf_size)) == -1 && errno == EINTR)
			;

		igt_assert_neq(len, -1);

		igt_assert(len > report_size * n_full_oa_reports * 0.45);
		igt_assert(len < report_size * n_full_oa_reports * 0.55);

		do_ioctl(stream_fd, I915_PERF_IOCTL_DISABLE, 0);

		/* It's considered an error to read a stream while it's disabled
		 * since it would block indefinitely...
		 */
		len = read(stream_fd, buf, buf_size);

		igt_assert_eq(len, -1);
		igt_assert_eq(errno, EIO);
	}

	free(buf);

	__perf_close(stream_fd);
}

static void
test_short_reads(void)
{
	int oa_exponent = max_oa_exponent_for_period_lte(5000);
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exponent,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	size_t record_size = 256 + sizeof(struct drm_i915_perf_record_header);
	size_t page_size = sysconf(_SC_PAGE_SIZE);
	int zero_fd = open("/dev/zero", O_RDWR|O_CLOEXEC);
	uint8_t *pages = mmap(NULL, page_size * 2,
			      PROT_READ|PROT_WRITE, MAP_PRIVATE, zero_fd, 0);
	struct drm_i915_perf_record_header *header;
	int ret;

	igt_assert_neq(zero_fd, -1);
	close(zero_fd);
	zero_fd = -1;

	igt_assert(pages);

	ret = mprotect(pages + page_size, page_size, PROT_NONE);
	igt_assert_eq(ret, 0);

	stream_fd = __perf_open(drm_fd, &param);

	nanosleep(&(struct timespec){ .tv_sec = 0, .tv_nsec = 5000000 }, NULL);

	/* At this point there should be lots of pending reports to read */

	/* A read that can return at least one record should result in a short
	 * read not an EFAULT if the buffer is smaller than the requested read
	 * size...
	 *
	 * Expect to see a sample record here, but at least skip over any
	 * _RECORD_LOST notifications.
	 */
	do {
		header = (void *)(pages + page_size - record_size);
		ret = read(stream_fd,
			   header,
			   page_size);
		igt_assert(ret > 0);
	} while (header->type == DRM_I915_PERF_RECORD_OA_REPORT_LOST);

	igt_assert_eq(ret, record_size);

	/* A read that can't return a single record because it would result
	 * in a fault on buffer overrun should result in an EFAULT error...
	 */
	ret = read(stream_fd, pages + page_size - 16, page_size);
	igt_assert_eq(ret, -1);
	igt_assert_eq(errno, EFAULT);

	/* A read that can't return a single record because the buffer is too
	 * small should result in an ENOSPC error..
	 *
	 * Again, skip over _RECORD_LOST records (smaller than record_size/2)
	 */
	do {
		header = (void *)(pages + page_size - record_size / 2);
		ret = read(stream_fd,
			   header,
			   record_size / 2);
	} while (ret > 0 && header->type == DRM_I915_PERF_RECORD_OA_REPORT_LOST);

	igt_assert_eq(ret, -1);
	igt_assert_eq(errno, ENOSPC);

	__perf_close(stream_fd);

	munmap(pages, page_size * 2);
}

static void
test_non_sampling_read_error(void)
{
	uint64_t properties[] = {
		/* XXX: even without periodic sampling we have to
		 * specify at least one sample layout property...
		 */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,

		/* XXX: no sampling exponent */
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	int ret;
	uint8_t buf[1024];

	stream_fd = __perf_open(drm_fd, &param);

	ret = read(stream_fd, buf, sizeof(buf));
	igt_assert_eq(ret, -1);
	igt_assert_eq(errno, EIO);

	__perf_close(stream_fd);
}

/* Check that attempts to read from a stream while it is disable will return
 * EIO instead of blocking indefinitely.
 */
static void
test_disabled_read_error(void)
{
	int oa_exponent = 5; /* 5 micro seconds */
	uint64_t properties[] = {
		/* XXX: even without periodic sampling we have to
		 * specify at least one sample layout property...
		 */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exponent,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC |
			 I915_PERF_FLAG_DISABLED, /* XXX: open disabled */
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	uint32_t oa_report0[64];
	uint32_t oa_report1[64];
	uint32_t buf[128] = { 0 };
	int ret;

	stream_fd = __perf_open(drm_fd, &param);

	ret = read(stream_fd, buf, sizeof(buf));
	igt_assert_eq(ret, -1);
	igt_assert_eq(errno, EIO);

	__perf_close(stream_fd);


	param.flags &= ~I915_PERF_FLAG_DISABLED;
	stream_fd = __perf_open(drm_fd, &param);

	while (read_2_oa_reports(test_oa_format,
				 oa_exponent,
				 oa_report0,
				 oa_report1,
				 false) == EAGAIN) /* not just timer reports */
		;

	do_ioctl(stream_fd, I915_PERF_IOCTL_DISABLE, 0);

	ret = read(stream_fd, buf, sizeof(buf));
	igt_assert_eq(ret, -1);
	igt_assert_eq(errno, EIO);

	do_ioctl(stream_fd, I915_PERF_IOCTL_ENABLE, 0);

	while (!read_2_oa_reports(test_oa_format,
				  oa_exponent,
				  oa_report0,
				  oa_report1,
				  false)) /* not just timer reports */
		;

	__perf_close(stream_fd);
}

static void
emit_report_perf_count(struct intel_batchbuffer *batch,
		       drm_intel_bo *dst_bo,
		       int dst_offset,
		       uint32_t report_id)
{
	if (IS_HASWELL(devid)) {
		BEGIN_BATCH(3, 1);
		OUT_BATCH(GEN6_MI_REPORT_PERF_COUNT);
		OUT_RELOC(dst_bo, I915_GEM_DOMAIN_INSTRUCTION, I915_GEM_DOMAIN_INSTRUCTION,
			  dst_offset);
		OUT_BATCH(report_id);
		ADVANCE_BATCH();
	} else {
		/* XXX: NB: n dwords arg is actually magic since it internally
		 * automatically accounts for larger addresses on gen >= 8...
		 */
		BEGIN_BATCH(3, 1);
		OUT_BATCH(GEN8_MI_REPORT_PERF_COUNT);
		OUT_RELOC(dst_bo, I915_GEM_DOMAIN_INSTRUCTION, I915_GEM_DOMAIN_INSTRUCTION,
			  dst_offset);
		OUT_BATCH(report_id);
		ADVANCE_BATCH();
	}
}

static void
test_mi_rpc(void)
{
	uint64_t properties[] = {
		/* Note: we have to specify at least one sample property even
		 * though we aren't interested in samples in this case.
		 */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,

		/* Note: no OA exponent specified in this case */
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	drm_intel_bufmgr *bufmgr = drm_intel_bufmgr_gem_init(drm_fd, 4096);
	drm_intel_context *context;
	struct intel_batchbuffer *batch;
	drm_intel_bo *bo;
	uint32_t *report32;
	int ret;

	stream_fd = __perf_open(drm_fd, &param);

	drm_intel_bufmgr_gem_enable_reuse(bufmgr);

	context = drm_intel_gem_context_create(bufmgr);
	igt_assert(context);

	batch = intel_batchbuffer_alloc(bufmgr, devid);

	bo = drm_intel_bo_alloc(bufmgr, "mi_rpc dest bo", 4096, 64);

	ret = drm_intel_bo_map(bo, true);
	igt_assert_eq(ret, 0);

	memset(bo->virtual, 0x80, 4096);
	drm_intel_bo_unmap(bo);

	emit_report_perf_count(batch,
			       bo, /* dst */
			       0, /* dst offset in bytes */
			       0xdeadbeef); /* report ID */

	intel_batchbuffer_flush_with_context(batch, context);

	ret = drm_intel_bo_map(bo, false /* write enable */);
	igt_assert_eq(ret, 0);

	report32 = bo->virtual;
	igt_assert_eq(report32[0], 0xdeadbeef); /* report ID */
	igt_assert_neq(report32[1], 0); /* timestamp */

	igt_assert_neq(report32[63], 0x80808080); /* end of report */
	igt_assert_eq(report32[64], 0x80808080); /* after 256 byte report */

	drm_intel_bo_unmap(bo);
	drm_intel_bo_unreference(bo);
	intel_batchbuffer_free(batch);
	drm_intel_gem_context_destroy(context);
	drm_intel_bufmgr_destroy(bufmgr);
	__perf_close(stream_fd);
}

static void
scratch_buf_init(drm_intel_bufmgr *bufmgr,
		 struct igt_buf *buf,
		 int width, int height,
		 uint32_t color)
{
	size_t stride = width * 4;
	size_t size = stride * height;
	drm_intel_bo *bo = drm_intel_bo_alloc(bufmgr, "", size, 4096);
	int ret;

	ret = drm_intel_bo_map(bo, true /* writable */);
	igt_assert_eq(ret, 0);

	for (int i = 0; i < width * height; i++)
		((uint32_t *)bo->virtual)[i] = color;

	drm_intel_bo_unmap(bo);

	buf->bo = bo;
	buf->stride = stride;
	buf->tiling = I915_TILING_NONE;
	buf->size = size;
}

static void
emit_stall_timestamp_and_rpc(struct intel_batchbuffer *batch,
			     drm_intel_bo *dst,
			     int timestamp_offset,
			     int report_dst_offset,
			     uint32_t report_id)
{
	uint32_t pipe_ctl_flags = (PIPE_CONTROL_CS_STALL |
				   PIPE_CONTROL_RENDER_TARGET_FLUSH |
				   PIPE_CONTROL_WRITE_TIMESTAMP);

	if (intel_gen(devid) >= 8) {
		BEGIN_BATCH(5, 1);
		OUT_BATCH(GFX_OP_PIPE_CONTROL | (6 - 2));
		OUT_BATCH(pipe_ctl_flags);
		OUT_RELOC(dst, I915_GEM_DOMAIN_INSTRUCTION, I915_GEM_DOMAIN_INSTRUCTION,
			  timestamp_offset);
		OUT_BATCH(0); /* imm lower */
		OUT_BATCH(0); /* imm upper */
		ADVANCE_BATCH();
	} else {
		BEGIN_BATCH(5, 1);
		OUT_BATCH(GFX_OP_PIPE_CONTROL | (5 - 2));
		OUT_BATCH(pipe_ctl_flags);
		OUT_RELOC(dst, I915_GEM_DOMAIN_INSTRUCTION, I915_GEM_DOMAIN_INSTRUCTION,
			  timestamp_offset);
		OUT_BATCH(0); /* imm lower */
		OUT_BATCH(0); /* imm upper */
		ADVANCE_BATCH();
	}

	emit_report_perf_count(batch, dst, report_dst_offset, report_id);
}

/* Tests the INTEL_performance_query use case where an unprivileged process
 * should be able to configure the OA unit for per-context metrics (for a
 * context associated with that process' drm file descriptor) and the counters
 * should only relate to that specific context.
 *
 * Unfortunately only Haswell limits the progression of OA counters for a
 * single context and so this unit test is Haswell specific. For Gen8+ although
 * reports read via i915 perf can be filtered for a single context the counters
 * themselves always progress as global/system-wide counters affected by all
 * contexts.
 */
static void
hsw_test_single_ctx_counters(void)
{
	uint64_t properties[] = {
		DRM_I915_PERF_PROP_CTX_HANDLE, UINT64_MAX, /* updated below */

		/* Note: we have to specify at least one sample property even
		 * though we aren't interested in samples in this case
		 */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,

		/* Note: no OA exponent specified in this case */
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};

	/* should be default, but just to be sure... */
	write_u64_file("/proc/sys/dev/i915/perf_stream_paranoid", 1);

	igt_fork(child, 1) {
		drm_intel_bufmgr *bufmgr;
		drm_intel_context *context0, *context1;
		struct intel_batchbuffer *batch;
		struct igt_buf src, dst;
		drm_intel_bo *bo;
		uint32_t *report0_32, *report1_32;
		uint64_t timestamp0_64, timestamp1_64;
		uint32_t delta_ts64, delta_oa32;
		uint64_t delta_ts64_ns, delta_oa32_ns;
		uint32_t delta_delta;
		int n_samples_written;
		int width = 800;
		int height = 600;
		uint32_t ctx_id = 0xffffffff; /* invalid id */
		int ret;

		igt_drop_root();

		bufmgr = drm_intel_bufmgr_gem_init(drm_fd, 4096);
		drm_intel_bufmgr_gem_enable_reuse(bufmgr);

		scratch_buf_init(bufmgr, &src, width, height, 0xff0000ff);
		scratch_buf_init(bufmgr, &dst, width, height, 0x00ff00ff);

		batch = intel_batchbuffer_alloc(bufmgr, devid);

		context0 = drm_intel_gem_context_create(bufmgr);
		igt_assert(context0);

		context1 = drm_intel_gem_context_create(bufmgr);
		igt_assert(context1);

		igt_debug("submitting warm up render_copy\n");

		/* Submit some early, unmeasured, work to the context we want
		 * to measure to try and catch issues with i915-perf
		 * initializing the HW context ID for filtering.
		 *
		 * We do this because i915-perf single context filtering had
		 * previously only relied on a hook into context pinning to
		 * initialize the HW context ID, instead of also trying to
		 * determine the HW ID while opening the stream, in case it
		 * has already been pinned.
		 *
		 * This wasn't noticed by the previous unit test because we
		 * were opening the stream while the context hadn't been
		 * touched or pinned yet and so it worked out correctly to wait
		 * for the pinning hook.
		 *
		 * Now a buggy version of i915-perf will fail to measure
		 * anything for context0 once this initial render_copy() ends
		 * up pinning the context since there won't ever be a pinning
		 * hook callback.
		 */
		render_copy(batch,
			    context0,
			    &src, 0, 0, width, height,
			    &dst, 0, 0);

		ret = drm_intel_gem_context_get_id(context0, &ctx_id);
		igt_assert_eq(ret, 0);
		igt_assert_neq(ctx_id, 0xffffffff);
		properties[1] = ctx_id;

		igt_debug("opening i915-perf stream\n");
		stream_fd = __perf_open(drm_fd, &param);

		bo = drm_intel_bo_alloc(bufmgr, "mi_rpc dest bo", 4096, 64);

		ret = drm_intel_bo_map(bo, true /* write enable */);
		igt_assert_eq(ret, 0);

		memset(bo->virtual, 0x80, 4096);
		drm_intel_bo_unmap(bo);

		emit_stall_timestamp_and_rpc(batch,
					     bo,
					     512 /* timestamp offset */,
					     0, /* report dst offset */
					     0xdeadbeef); /* report id */

		/* Explicitly flush here (even though the render_copy() call
		 * will itself flush before/after the copy) to clarify that
		 * that the PIPE_CONTROL + MI_RPC commands will be in a
		 * separate batch from the copy.
		 */
		intel_batchbuffer_flush_with_context(batch, context0);

		render_copy(batch,
			    context0,
			    &src, 0, 0, width, height,
			    &dst, 0, 0);

		/* Another redundant flush to clarify batch bo is free to reuse */
		intel_batchbuffer_flush_with_context(batch, context0);

		/* submit two copies on the other context to avoid a false
		 * positive in case the driver somehow ended up filtering for
		 * context1
		 */
		render_copy(batch,
			    context1,
			    &src, 0, 0, width, height,
			    &dst, 0, 0);

		render_copy(batch,
			    context1,
			    &src, 0, 0, width, height,
			    &dst, 0, 0);

		/* And another */
		intel_batchbuffer_flush_with_context(batch, context1);

		emit_stall_timestamp_and_rpc(batch,
					     bo,
					     520 /* timestamp offset */,
					     256, /* report dst offset */
					     0xbeefbeef); /* report id */

		intel_batchbuffer_flush_with_context(batch, context0);

		ret = drm_intel_bo_map(bo, false /* write enable */);
		igt_assert_eq(ret, 0);

		report0_32 = bo->virtual;
		igt_assert_eq(report0_32[0], 0xdeadbeef); /* report ID */
		igt_assert_neq(report0_32[1], 0); /* timestamp */

		report1_32 = report0_32 + 64;
		igt_assert_eq(report1_32[0], 0xbeefbeef); /* report ID */
		igt_assert_neq(report1_32[1], 0); /* timestamp */

		print_reports(report0_32, report1_32,
			      lookup_format(test_oa_format));

		/* A40 == N samples written to all render targets */
		n_samples_written = report1_32[43] - report0_32[43];
		igt_debug("n samples written = %d\n", n_samples_written);
		igt_assert_eq(n_samples_written, width * height);

		igt_debug("timestamp32 0 = %u\n", report0_32[1]);
		igt_debug("timestamp32 1 = %u\n", report1_32[1]);

		timestamp0_64 = *(uint64_t *)(((uint8_t *)bo->virtual) + 512);
		timestamp1_64 = *(uint64_t *)(((uint8_t *)bo->virtual) + 520);

		igt_debug("timestamp64 0 = %"PRIu64"\n", timestamp0_64);
		igt_debug("timestamp64 1 = %"PRIu64"\n", timestamp1_64);

		delta_ts64 = timestamp1_64 - timestamp0_64;
		delta_oa32 = report1_32[1] - report0_32[1];

		/* sanity check that we can pass the delta to timebase_scale */
		igt_assert(delta_ts64 < UINT32_MAX);
		delta_oa32_ns = timebase_scale(delta_oa32);
		delta_ts64_ns = timebase_scale(delta_ts64);

		igt_debug("ts32 delta = %u, = %uns\n",
			  delta_oa32, (unsigned)delta_oa32_ns);
		igt_debug("ts64 delta = %u, = %uns\n",
			  delta_ts64, (unsigned)delta_ts64_ns);

		/* The delta as calculated via the PIPE_CONTROL timestamp or
		 * the OA report timestamps should be almost identical but
		 * allow a 320 nanoseconds margin.
		 */
		delta_delta = delta_ts64_ns > delta_oa32_ns ?
			(delta_ts64_ns - delta_oa32_ns) :
			(delta_oa32_ns - delta_ts64_ns);
		igt_assert(delta_delta <= 320);

		drm_intel_bo_unreference(src.bo);
		drm_intel_bo_unreference(dst.bo);

		drm_intel_bo_unmap(bo);
		drm_intel_bo_unreference(bo);
		intel_batchbuffer_free(batch);
		drm_intel_gem_context_destroy(context0);
		drm_intel_gem_context_destroy(context1);
		drm_intel_bufmgr_destroy(bufmgr);
		__perf_close(stream_fd);
	}

	igt_waitchildren();
}

/* Tests the INTEL_performance_query use case where an unprivileged process
 * should be able to configure the OA unit for per-context metrics (for a
 * context associated with that process' drm file descriptor) and the counters
 * should only relate to that specific context.
 *
 * For Gen8+ although reports read via i915 perf can be filtered for a single
 * context the counters themselves always progress as global/system-wide
 * counters affected by all contexts. To support the INTEL_performance_query
 * use case on Gen8+ it's necessary to combine OABUFFER and
 * MI_REPORT_PERF_COUNT reports so that counter normalisation can take into
 * account context-switch reports and factor out any counter progression not
 * associated with the current context.
 */
static void
gen8_test_single_ctx_render_target_writes_a_counter(void)
{
	int oa_exponent = max_oa_exponent_for_period_lte(1000000);
	uint64_t properties[] = {
		DRM_I915_PERF_PROP_CTX_HANDLE, UINT64_MAX, /* updated below */

		/* Note: we have to specify at least one sample property even
		 * though we aren't interested in samples in this case
		 */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exponent,

		/* Note: no OA exponent specified in this case */
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	size_t format_size = oa_formats[test_oa_format].size;
	size_t sample_size = (sizeof(struct drm_i915_perf_record_header) +
			      format_size);
	int max_reports = (16 * 1024 * 1024) / format_size;
	int buf_size = sample_size * max_reports * 1.5;
	uint8_t *buf = malloc(buf_size);
	struct drm_i915_perf_record_header *header;
	ssize_t len;

	/* should be default, but just to be sure... */
	write_u64_file("/proc/sys/dev/i915/perf_stream_paranoid", 1);

	igt_fork(child, 1) {
		drm_intel_bufmgr *bufmgr;
		drm_intel_context *context0, *context1;
		struct intel_batchbuffer *batch;
		struct igt_buf src, dst;
		drm_intel_bo *bo;
		uint32_t *report0_32, *report1_32;
		uint32_t *prev;
		uint64_t timestamp0_64, timestamp1_64;
		uint32_t delta_ts64, delta_oa32;
		uint64_t delta_ts64_ns, delta_oa32_ns;
		uint32_t delta_delta;
		int width = 800;
		int height = 600;
		uint32_t ctx_id = 0xffffffff; /* invalid handle */
		bool in_ctx = true;
		int ret;
		struct counters_record *records =
			counters_record_allocate(test_oa_format),
			*all_records = counters_record_allocate(test_oa_format);

		//igt_drop_root();

		bufmgr = drm_intel_bufmgr_gem_init(drm_fd, 4096);
		drm_intel_bufmgr_gem_enable_reuse(bufmgr);

		scratch_buf_init(bufmgr, &src, width, height, 0xff0000ff);
		scratch_buf_init(bufmgr, &dst, width, height, 0x00ff00ff);

		batch = intel_batchbuffer_alloc(bufmgr, devid);

		context0 = drm_intel_gem_context_create(bufmgr);
		igt_assert(context0);

		context1 = drm_intel_gem_context_create(bufmgr);
		igt_assert(context1);

		igt_debug("submitting warm up render_copy\n");

		/* Submit some early, unmeasured, work to the context we want
		 * to measure to try and catch issues with i915-perf
		 * initializing the HW context ID for filtering.
		 *
		 * We do this because i915-perf single context filtering had
		 * previously only relied on a hook into context pinning to
		 * initialize the HW context ID, instead of also trying to
		 * determine the HW ID while opening the stream, in case it
		 * has already been pinned.
		 *
		 * This wasn't noticed by the previous unit test because we
		 * were opening the stream while the context hadn't been
		 * touched or pinned yet and so it worked out correctly to wait
		 * for the pinning hook.
		 *
		 * Now a buggy version of i915-perf will fail to measure
		 * anything for context0 once this initial render_copy() ends
		 * up pinning the context since there won't ever be a pinning
		 * hook callback.
		 */
		render_copy(batch,
			    context0,
			    &src, 0, 0, width, height,
			    &dst, 0, 0);

		ret = drm_intel_gem_context_get_id(context0, &ctx_id);
		igt_assert_eq(ret, 0);
		igt_assert_neq(ctx_id, 0xffffffff);
		properties[1] = ctx_id;

		igt_debug("opening i915-perf stream\n");
		stream_fd = __perf_open(drm_fd, &param);

		bo = drm_intel_bo_alloc(bufmgr, "mi_rpc dest bo", 4096, 64);

		ret = drm_intel_bo_map(bo, true /* write enable */);
		igt_assert_eq(ret, 0);

		memset(bo->virtual, 0x80, 4096);
		drm_intel_bo_unmap(bo);

		emit_stall_timestamp_and_rpc(batch,
					     bo,
					     512 /* timestamp offset */,
					     0, /* report dst offset */
					     0xdeadbeef); /* report id */

		/* Explicitly flush here (even though the render_copy() call
		 * will itself flush before/after the copy) to clarify that
		 * that the PIPE_CONTROL + MI_RPC commands will be in a
		 * separate batch from the copy.
		 */
		intel_batchbuffer_flush_with_context(batch, context0);

		render_copy(batch,
			    context0,
			    &src, 0, 0, width, height,
			    &dst, 0, 0);

		/* Another redundant flush to clarify batch bo is free to reuse */
		intel_batchbuffer_flush_with_context(batch, context0);

		/* submit two copies on the other context to avoid a false
		 * positive in case the driver somehow ended up filtering for
		 * context1
		 */
		render_copy(batch,
			    context1,
			    &src, 0, 0, width, height,
			    &dst, 0, 0);

		render_copy(batch,
			    context1,
			    &src, 0, 0, width, height,
			    &dst, 0, 0);

		/* And another */
		intel_batchbuffer_flush_with_context(batch, context1);

		emit_stall_timestamp_and_rpc(batch,
					     bo,
					     520 /* timestamp offset */,
					     256, /* report dst offset */
					     0xbeefbeef); /* report id */

		intel_batchbuffer_flush_with_context(batch, context1);

		ret = drm_intel_bo_map(bo, false /* write enable */);
		igt_assert_eq(ret, 0);

		report0_32 = bo->virtual;
		igt_assert_eq(report0_32[0], 0xdeadbeef); /* report ID */
		igt_assert_neq(report0_32[1], 0); /* timestamp */
		//report0_32[2] = 0xffffffff;
		prev = report0_32;
		ctx_id = prev[2];
		in_ctx = true;
		igt_debug("MI_RPC(start) CTX ID: %u\n", ctx_id);

		report1_32 = report0_32 + 64; /* 64 uint32_t = 256bytes offset */
		igt_assert_eq(report1_32[0], 0xbeefbeef); /* report ID */
		igt_assert_neq(report1_32[1], 0); /* timestamp */
		//report1_32[2] = 0xffffffff;

		counters_record_reset(all_records);
		counters_record_update(all_records, report0_32, report1_32);
		igt_debug("total: A40_0 = %lu, A40_21 = %lu, A40_26 = %lu\n",
			  all_records->a40_records[0],
			  all_records->a40_records[21],
			  all_records->a40_records[26]);

		igt_debug("oa_timestamp32 0 = %u\n", report0_32[1]);
		igt_debug("oa_timestamp32 1 = %u\n", report1_32[1]);

		timestamp0_64 = *(uint64_t *)(((uint8_t *)bo->virtual) + 512);
		timestamp1_64 = *(uint64_t *)(((uint8_t *)bo->virtual) + 520);

		igt_debug("ts_timestamp64 0 = %"PRIu64"\n", timestamp0_64);
		igt_debug("ts_timestamp64 1 = %"PRIu64"\n", timestamp1_64);

		delta_ts64 = timestamp1_64 - timestamp0_64;
		delta_oa32 = report1_32[1] - report0_32[1];

		/* sanity check that we can pass the delta to timebase_scale */
		igt_assert(delta_ts64 < UINT32_MAX);
		delta_oa32_ns = timebase_scale(delta_oa32);
		delta_ts64_ns = timebase_scale(delta_ts64);

		igt_debug("oa32 delta = %u, = %uns\n",
			  delta_oa32, (unsigned)delta_oa32_ns);
		igt_debug("ts64 delta = %u, = %uns\n",
			  delta_ts64, (unsigned)delta_ts64_ns);

		/* The delta as calculated via the PIPE_CONTROL timestamp or
		 * the OA report timestamps should be almost identical but
		 * allow a 320 nanoseconds margin.
		 */
		delta_delta = delta_ts64_ns > delta_oa32_ns ?
			(delta_ts64_ns - delta_oa32_ns) :
			(delta_oa32_ns - delta_ts64_ns);
		igt_assert(delta_delta <= 500);


		len = i915_read_reports_until_timestamp(test_oa_format,
							buf, buf_size,
							report0_32[1],
							report1_32[1]);

		igt_assert(len > 0);
		igt_debug("read %d bytes\n", (int)len);

		for (size_t offset = 0; offset < len; offset += header->size) {
			uint32_t *report;
			uint32_t reason;
			bool skip = false;

			header = (void *)(buf + offset);

			igt_assert_eq(header->pad, 0); /* Reserved */

			/* Currently the only test that should ever expect to
			 * see a _BUFFER_LOST error is the buffer_fill test,
			 * otherwise something bad has probably happened...
			 */
			igt_assert_neq(header->type, DRM_I915_PERF_RECORD_OA_BUFFER_LOST);

			/* At high sampling frequencies the OA HW might not be
			 * able to cope with all write requests and will notify
			 * us that a report was lost.
			 *
			 * XXX: we should maybe restart the test in this case?
			 */
			if (header->type == DRM_I915_PERF_RECORD_OA_REPORT_LOST) {
				igt_debug("OA trigger collision / report lost\n");
				continue;
			}

			/* Currently the only other record type expected is a
			 * _SAMPLE. Notably this test will need updating if
			 * i915-perf is extended in the future with additional
			 * record types.
			 */
			igt_assert_eq(header->type, DRM_I915_PERF_RECORD_SAMPLE);

			igt_assert_eq(header->size, sample_size);

			report = (void *)(header + 1);

			counters_record_reset(all_records);
			counters_record_update(all_records, prev, report);
			igt_debug("read report %p: reason = %x, timestamp = %u\n",
				  report, report[0], report[1]);

			/*
			 * Annoying cases to consider:
			 *
			 * back to back switches for the specific ctx:
			 *	- are they away, then to, or to then away?
			 *	- we need to disambiguate switching two or from
			 */

			/* Don't expect zero for timestamps */
			igt_assert_neq(report[1], 0);

			{
				uint32_t time_delta = report[1] - report0_32[1];

				if (timebase_scale(time_delta) > 1000000000) {
					igt_debug("Skipping report earlier than first MI_RPC (%u)\n", prev[1]);
					continue;
				}
			}

			{
				uint32_t time_delta = report[1] - report1_32[1];

				if (timebase_scale(time_delta) <= 1000000000) {
					igt_debug("Report comes after last MI_RPC (%u)\n", report1_32[1]);
					report = report1_32;
				}
			}


			igt_debug(" ctx ID: %u/%x\n", report[2], report[2]);

			reason = ((report[0] >> OAREPORT_REASON_SHIFT) &
				  OAREPORT_REASON_MASK);

			if (in_ctx && report[2] != ctx_id) {
				igt_debug(" Switch AWAY (observed by ID change)\n");
				in_ctx = false;
			} else if (in_ctx == false &&
				   report[2] == ctx_id && (reason & OAREPORT_REASON_CTX_SWITCH)) {
				/* Consider that we might have already observed
				 * a switch-to via a periodic report...
				 */

				/* Note it's possible to see sequences of
				 * switch-to reports since there's no
				 * switch-away report triggered by the hardware
				 * and the same context may be scheduled back
				 * to back. In this case we will accumulate
				 * between sequential switch-away reports.
				 */
				igt_debug(" Switch TO (report reason = CTX_SWITCH)\n");
				in_ctx = true;
				skip = true;
			} else if (in_ctx == false && report[2] == ctx_id) {
				/* Note: here we're being careful to consider
				 * the possibility of a _CTX_SWITCH switch-away
				 * (with matching ctx_id) followed by a periodic/timer
				 * report (also with matching ctx_id) due to
				 * repeat scheduling of the same context
				 */
				igt_debug(" Switch TO (observed by ID change)\n");
				in_ctx = true;
				skip = true;
			} else if (in_ctx) {
				igt_assert_eq(report[2], ctx_id);
				igt_debug(" CONTINUATION IN\n");
			} else {
				igt_assert_eq(in_ctx, false);
				//igt_assert_neq(prev[2], ctx_id);
				igt_assert_neq(report[2], ctx_id);
				igt_debug(" CONTINUATION OUT\n");
				skip = true;
			}

			if (reason & OAREPORT_REASON_CTX_SWITCH) {
				igt_debug(" reason: ctx-switch\n");
			} else if (reason & OAREPORT_REASON_TIMER) {
				igt_debug(" reason: timer\n");
			}

			/* Don't accumulate deltas in between context switch
			 * reports, where counter increases belong to other
			 * contexts
			 */
			if (!skip) {
				counters_record_update(records, prev, report);
			}

			prev = report;

			if (report == report1_32)
				break;
		}

		igt_debug("n samples written = %ld/%lu (%ix%i)\n",
			  records->a40_records[21],
			  records->a40_records[26],
			  width, height);
		counters_record_print(records, "filtered");
		igt_assert_eq(records->a40_records[26], width * height);

		counters_record_free(records);

		drm_intel_bo_unreference(src.bo);
		drm_intel_bo_unreference(dst.bo);

		drm_intel_bo_unmap(bo);
		drm_intel_bo_unreference(bo);
		intel_batchbuffer_free(batch);
		drm_intel_gem_context_destroy(context0);
		drm_intel_gem_context_destroy(context1);
		drm_intel_bufmgr_destroy(bufmgr);
		__perf_close(stream_fd);
	}

	igt_waitchildren();
}

static void
test_rc6_disable(void)
{
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, test_metric_set_id,
		DRM_I915_PERF_PROP_OA_FORMAT, test_oa_format,
		DRM_I915_PERF_PROP_OA_EXPONENT, oa_exp_1_millisec,
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	uint64_t n_events_start, n_events_end;

	stream_fd = __perf_open(drm_fd, &param);

	n_events_start = read_debugfs_u64_record("i915_drpc_info",
						 "RC6 residency since boot");

	nanosleep(&(struct timespec){ .tv_sec = 0, .tv_nsec = 500000000 }, NULL);

	n_events_end = read_debugfs_u64_record("i915_drpc_info",
					       "RC6 residency since boot");

	igt_assert_eq(n_events_end - n_events_start, 0);

	__perf_close(stream_fd);

	n_events_start = read_debugfs_u64_record("i915_drpc_info",
						 "RC6 residency since boot");

	nanosleep(&(struct timespec){ .tv_sec = 0, .tv_nsec = 500000000 }, NULL);

	n_events_end = read_debugfs_u64_record("i915_drpc_info",
					       "RC6 residency since boot");

	igt_assert_neq(n_events_end - n_events_start, 0);
}

static unsigned
read_i915_module_ref(void)
{
	FILE *fp = fopen("/proc/modules", "r");
	char *line = NULL;
	size_t line_buf_size = 0;
	int len = 0;
	unsigned ref_count;

	igt_assert(fp);

	while ((len = getline(&line, &line_buf_size, fp)) > 0) {
		if (strncmp(line, "i915 ", 5) == 0) {
			unsigned long mem;
			int ret = sscanf(line + 5, "%lu %u", &mem, &ref_count);
			igt_assert(ret == 2);
			goto done;
		}
	}

	igt_assert(!"reached");

done:
	free(line);
	fclose(fp);
	return ref_count;
}

/* check that an open i915 perf stream holds a reference on the drm i915 module
 * including in the corner case where the original drm fd has been closed.
 */
static void
test_i915_ref_count(void)
{
	uint64_t properties[] = {
		/* Include OA reports in samples */
		DRM_I915_PERF_PROP_SAMPLE_OA, true,

		/* OA unit configuration */
		DRM_I915_PERF_PROP_OA_METRICS_SET, 0 /* updated below */,
		DRM_I915_PERF_PROP_OA_FORMAT, 0, /* update below */
		DRM_I915_PERF_PROP_OA_EXPONENT, 0, /* update below */
	};
	struct drm_i915_perf_open_param param = {
		.flags = I915_PERF_FLAG_FD_CLOEXEC,
		.num_properties = sizeof(properties) / 16,
		.properties_ptr = to_user_pointer(properties),
	};
	unsigned baseline, ref_count0, ref_count1;
	uint32_t oa_report0[64];
	uint32_t oa_report1[64];

	/* This should be the first test before the first fixture so no drm_fd
	 * should have been opened so far...
	 */
	igt_assert_eq(drm_fd, -1);

	baseline = read_i915_module_ref();
	igt_debug("baseline ref count (drm fd closed) = %u\n", baseline);

	drm_fd = __drm_open_driver(DRIVER_INTEL);
	devid = intel_get_drm_devid(drm_fd);
	device = drm_get_card();

	/* Note: these global variables are only initialized after calling
	 * init_sys_info()...
	 */
	igt_require(init_sys_info());
	properties[3] = test_metric_set_id;
	properties[5] = test_oa_format;
	properties[7] = oa_exp_1_millisec;

	ref_count0 = read_i915_module_ref();
	igt_debug("initial ref count with drm_fd open = %u\n", ref_count0);
	igt_assert(ref_count0 > baseline);

	stream_fd = __perf_open(drm_fd, &param);
	ref_count1 = read_i915_module_ref();
	igt_debug("ref count after opening i915 perf stream = %u\n", ref_count1);
	igt_assert(ref_count1 > ref_count0);

	close(drm_fd);
	drm_fd = -1;
	ref_count0 = read_i915_module_ref();
	igt_debug("ref count after closing drm fd = %u\n", ref_count0);

	igt_assert(ref_count0 > baseline);

	read_2_oa_reports(test_oa_format,
			  oa_exp_1_millisec,
			  oa_report0,
			  oa_report1,
			  false); /* not just timer reports */

	__perf_close(stream_fd);
	ref_count0 = read_i915_module_ref();
	igt_debug("ref count after closing i915 perf stream fd = %u\n", ref_count0);
	igt_assert_eq(ref_count0, baseline);
}

static void
test_sysctl_defaults(void)
{
	int paranoid = read_u64_file("/proc/sys/dev/i915/perf_stream_paranoid");
	int max_freq = read_u64_file("/proc/sys/dev/i915/oa_max_sample_rate");

	igt_assert_eq(paranoid, 1);
	igt_assert_eq(max_freq, 100000);
}

igt_main
{
	igt_skip_on_simulation();

	igt_fixture {
		struct stat sb;

		igt_require(stat("/proc/sys/dev/i915/perf_stream_paranoid", &sb)
			    == 0);
		igt_require(stat("/proc/sys/dev/i915/oa_max_sample_rate", &sb)
			    == 0);
	}

	igt_subtest("i915-ref-count")
		test_i915_ref_count();

	igt_subtest("sysctl-defaults")
		test_sysctl_defaults();

	igt_fixture {
		/* We expect that the ref count test before these fixtures
		 * should have closed drm_fd...
		 */
		igt_assert_eq(drm_fd, -1);
		drm_fd = drm_open_driver_render(DRIVER_INTEL);
		devid = intel_get_drm_devid(drm_fd);
		device = drm_get_card();

		igt_require(init_sys_info());

		gt_frequency_range_save();

		write_u64_file("/proc/sys/dev/i915/perf_stream_paranoid", 1);
		write_u64_file("/proc/sys/dev/i915/oa_max_sample_rate", 100000);

		render_copy = igt_get_render_copyfunc(devid);
		igt_require_f(render_copy, "no render-copy function\n");
	}

	igt_subtest("non-system-wide-paranoid")
		test_system_wide_paranoid();

	igt_subtest("invalid-open-flags")
		test_invalid_open_flags();

	igt_subtest("invalid-oa-metric-set-id")
		test_invalid_oa_metric_set_id();

	igt_subtest("invalid-oa-format-id")
		test_invalid_oa_format_id();

	igt_subtest("missing-sample-flags")
		test_missing_sample_flags();

	igt_subtest("oa-formats")
		test_oa_formats();

	igt_subtest("invalid-oa-exponent")
		test_invalid_oa_exponent();
	igt_subtest("low-oa-exponent-permissions")
		test_low_oa_exponent_permissions();
	igt_subtest("oa-exponents") {
		test_oa_exponents(450);
		test_oa_exponents(550);
	}

	igt_subtest("per-context-mode-unprivileged") {
		igt_require(IS_HASWELL(devid));
		test_per_context_mode_unprivileged();
	}

	igt_subtest("buffer-fill")
		test_buffer_fill();

	igt_subtest("disabled-read-error")
		test_disabled_read_error();
	igt_subtest("non-sampling-read-error")
		test_non_sampling_read_error();

	igt_subtest("enable-disable")
		test_enable_disable();

	igt_subtest("blocking")
		test_blocking();

	igt_subtest("polling")
		test_polling();

	igt_subtest("short-reads")
		test_short_reads();

	igt_subtest("mi-rpc")
		test_mi_rpc();

	igt_subtest("unprivileged-single-ctx-counters") {
		/* For Gen8+ the OA unit can no longer be made to clock gate
		 * for a specific context. Additionally the partial-replacement
		 * functionality to HW filter timer reports for a specific
		 * context (SKL+) can't stop multiple applications viewing
		 * system-wide data via MI_REPORT_PERF_COUNT commands.
		 */
		igt_require(IS_HASWELL(devid));
		hsw_test_single_ctx_counters();
	}

	igt_subtest("gen8-unprivileged-single-ctx-counters") {
		/* For Gen8+ the OA unit can no longer be made to clock gate
		 * for a specific context. Additionally the partial-replacement
		 * functionality to HW filter timer reports for a specific
		 * context (SKL+) can't stop multiple applications viewing
		 * system-wide data via MI_REPORT_PERF_COUNT commands.
		 */
		igt_require(intel_gen(devid) >= 8);
		gen8_test_single_ctx_render_target_writes_a_counter();
	}

	igt_subtest("rc6-disable")
		test_rc6_disable();

	igt_fixture {
		/* leave sysctl options in their default state... */
		write_u64_file("/proc/sys/dev/i915/oa_max_sample_rate", 100000);
		write_u64_file("/proc/sys/dev/i915/perf_stream_paranoid", 1);

		gt_frequency_range_restore();

		close(drm_fd);
	}
}
