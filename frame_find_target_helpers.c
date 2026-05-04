// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jerome ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jerome ESSOLA ELANGA

#include "frame_find_target_helpers.h"

#define BLUE_MIN_B5_LOW               3
#define BLUE_MIN_B5_HIGH             10
#define BLUE_MIN_BRIGHTNESS_LOW       4
#define BLUE_MIN_BRIGHTNESS_HIGH     18
#define BLUE_RED_MARGIN_LOW           2
#define BLUE_RED_MARGIN_HIGH          5
#define BLUE_GREEN_MARGIN_LOW         2
#define BLUE_GREEN_MARGIN_HIGH        4
#define BLUE_SCORE_LOW                5
#define BLUE_SCORE_HIGH               9
#define BLUE_RATIO_NUM               45
#define BLUE_RATIO_DEN              100
#define BLUE_SAMPLE_STEP              4

#define TARGET_ROI_TOP_NUM            1
#define TARGET_ROI_TOP_DEN            6
#define TARGET_ROI_BOTTOM_NUM         1
#define TARGET_ROI_BOTTOM_DEN         2

#define MIN_BLUE_SEGMENT_WIDTH        4
#define MAX_NON_BLUE_GAP_IN_SEGMENT   2
#define MAX_SEGMENTS_PER_ROW         12

#define MAX_ACTIVE_TARGETS           FRAME_FIND_TARGET_MAX_TARGETS
#define MAX_TARGET_CENTER_JUMP       12
#define MAX_TARGET_ROW_GAP            4
#define MIN_TARGET_HEIGHT             8
#define MIN_TARGET_SEGMENTS           6
#define MIN_TARGET_AREA              40
#define TARGET_OVERLAP_REJECT_NUM     1
#define TARGET_OVERLAP_REJECT_DEN     3

typedef struct {
    bool active;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int last_row;
    int last_center_x;
    int segment_count;
    int area;
    long sum_center_x;
    long sum_center_y;
} target_candidate_t;

typedef struct {
    int min_b5;
    int min_brightness;
    int red_margin;
    int green_margin;
    int min_blue_score;
} blue_filter_params_t;

static void compute_target_roi(int height, int *roi_top, int *roi_bottom);
static void clean_blue_rows_in_roi(uint16_t *frame, int width, int height);

static inline int rotm90_xo(int xv, int yv)
{
    (void)xv;
    return yv;
}

static inline int rotm90_yo(int xv, int height)
{
    return (height - 1) - xv;
}

static inline uint16_t get_px_rotm90(const uint16_t *frame, int width, int height, int xv, int yv)
{
    const int xo = rotm90_xo(xv, yv);
    const int yo = rotm90_yo(xv, height);
    return frame[yo * width + xo];
}

static inline void set_px_rotm90(uint16_t *frame, int width, int height, int xv, int yv, uint16_t color)
{
    const int xo = rotm90_xo(xv, yv);
    const int yo = rotm90_yo(xv, height);
    frame[yo * width + xo] = color;
}

static inline void rgb565_to_components(uint16_t px,
                                        uint8_t *r5,
                                        uint8_t *g6,
                                        uint8_t *b5)
{
    *r5 = (px >> 11) & 0x1F;
    *g6 = (px >> 5) & 0x3F;
    *b5 = px & 0x1F;
}

static int iabs_int(int x)
{
    return (x < 0) ? -x : x;
}

static int min_int(int a, int b)
{
    return (a < b) ? a : b;
}

static int max_int(int a, int b)
{
    return (a > b) ? a : b;
}

static int clamp_int(int x, int xmin, int xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static int overlap_len(int a0, int a1, int b0, int b1)
{
    const int start = max_int(a0, b0);
    const int end = min_int(a1, b1);

    if (end < start) {
        return 0;
    }

    return end - start + 1;
}

static void compute_target_roi(int height, int *roi_top, int *roi_bottom)
{
    int top = (height * TARGET_ROI_TOP_NUM) / TARGET_ROI_TOP_DEN;
    int bottom = (height * TARGET_ROI_BOTTOM_NUM) / TARGET_ROI_BOTTOM_DEN;

    if (top < 0) top = 0;
    if (bottom >= height) bottom = height - 1;
    if (bottom < top) bottom = top;

    *roi_top = top;
    *roi_bottom = bottom;
}

static blue_filter_params_t compute_blue_filter_params(const uint16_t *frame, int width, int height)
{
    long brightness_sum = 0;
    int sample_count = 0;
    blue_filter_params_t params;
    const int rotated_width = height;
    const int rotated_height = width;
    int roi_top;
    int roi_bottom;

    compute_target_roi(rotated_height, &roi_top, &roi_bottom);

    for (int y = roi_top; y <= roi_bottom; y += BLUE_SAMPLE_STEP) {
        for (int x = 0; x < rotated_width; x += BLUE_SAMPLE_STEP) {
            uint8_t r5, g6, b5;
            rgb565_to_components(get_px_rotm90(frame, width, height, x, y), &r5, &g6, &b5);

            brightness_sum += (int)r5 + ((int)g6 >> 1) + (int)b5;
            sample_count++;
        }
    }

    if (sample_count <= 0) {
        params.min_b5 = BLUE_MIN_B5_HIGH;
        params.min_brightness = BLUE_MIN_BRIGHTNESS_HIGH;
        params.red_margin = BLUE_RED_MARGIN_HIGH;
        params.green_margin = BLUE_GREEN_MARGIN_HIGH;
        params.min_blue_score = BLUE_SCORE_HIGH;
        return params;
    }

    {
        const int avg_brightness = (int)(brightness_sum / sample_count);

        params.min_b5 = clamp_int(avg_brightness / 7,
                                  BLUE_MIN_B5_LOW,
                                  BLUE_MIN_B5_HIGH);
        params.min_brightness = clamp_int(avg_brightness / 4,
                                          BLUE_MIN_BRIGHTNESS_LOW,
                                          BLUE_MIN_BRIGHTNESS_HIGH);
        params.red_margin = clamp_int(avg_brightness / 12,
                                      BLUE_RED_MARGIN_LOW,
                                      BLUE_RED_MARGIN_HIGH);
        params.green_margin = clamp_int(avg_brightness / 14,
                                        BLUE_GREEN_MARGIN_LOW,
                                        BLUE_GREEN_MARGIN_HIGH);
        params.min_blue_score = clamp_int(avg_brightness / 8,
                                          BLUE_SCORE_LOW,
                                          BLUE_SCORE_HIGH);
    }

    return params;
}

static bool is_blue_pixel(uint16_t px, const blue_filter_params_t *params)
{
    uint8_t r5, g6, b5;
    rgb565_to_components(px, &r5, &g6, &b5);

    const int g5 = (int)g6 >> 1;
    const int brightness = (int)r5 + g5 + (int)b5;
    const int blue_score = ((int)b5 * 2) - (int)r5 - g5;
    const bool blue_ratio_ok = ((int)b5 * BLUE_RATIO_DEN) >=
                               (brightness * BLUE_RATIO_NUM);

    if (brightness < params->min_brightness) {
        return false;
    }

    if (b5 < params->min_b5) {
        return false;
    }

    if (((int)b5 - (int)r5) < params->red_margin) {
        return false;
    }

    if (((int)b5 - g5) < params->green_margin) {
        return false;
    }

    if (blue_score < params->min_blue_score && !blue_ratio_ok) {
        return false;
    }

    return true;
}

void filter_blue_pxl(uint16_t *frame, int width, int height)
{
    const blue_filter_params_t params = compute_blue_filter_params(frame, width, height);
    const int rotated_width = height;
    const int rotated_height = width;
    int roi_top;
    int roi_bottom;

    compute_target_roi(rotated_height, &roi_top, &roi_bottom);

    for (int y = 0; y < rotated_height; ++y) {
        for (int x = 0; x < rotated_width; ++x) {
            if (y < roi_top || y > roi_bottom) {
                set_px_rotm90(frame, width, height, x, y, TARGET_COLOR_WHITE);
                continue;
            }

            set_px_rotm90(frame,
                          width,
                          height,
                          x,
                          y,
                          is_blue_pixel(get_px_rotm90(frame, width, height, x, y), &params)
                              ? TARGET_COLOR_BLUE
                              : TARGET_COLOR_WHITE);
        }
    }

    clean_blue_rows_in_roi(frame, width, height);
}

static void register_segment(target_row_segment_t *segments,
                             int *segment_count,
                             int row,
                             int start_x,
                             int end_x)
{
    if (*segment_count >= MAX_SEGMENTS_PER_ROW) {
        return;
    }

    segments[*segment_count].start_x = start_x;
    segments[*segment_count].end_x = end_x;
    segments[*segment_count].center_x = (start_x + end_x) / 2;
    segments[*segment_count].width = end_x - start_x + 1;
    segments[*segment_count].row = row;
    (*segment_count)++;
}

int find_blue_segments_in_row(const uint16_t *frame,
                              int width,
                              int height,
                              int row,
                              target_row_segment_t *segments,
                              int max_segments)
{
    int segment_start = -1;
    int last_blue_x = -1;
    int non_blue_gap = 0;
    int segment_count = 0;
    const int rotated_width = height;
    const int rotated_height = width;

    if (!frame || !segments || row < 0 || row >= rotated_height || max_segments <= 0) {
        return 0;
    }

    if (max_segments > MAX_SEGMENTS_PER_ROW) {
        max_segments = MAX_SEGMENTS_PER_ROW;
    }

    for (int x = 0; x < rotated_width; ++x) {
        const bool is_blue = (get_px_rotm90(frame, width, height, x, row) == TARGET_COLOR_BLUE);

        if (is_blue) {
            if (segment_start < 0) {
                segment_start = x;
            }

            last_blue_x = x;
            non_blue_gap = 0;
            continue;
        }

        if (segment_start < 0) {
            continue;
        }

        non_blue_gap++;
        if (non_blue_gap <= MAX_NON_BLUE_GAP_IN_SEGMENT) {
            continue;
        }

        if (last_blue_x >= segment_start) {
            const int segment_width = last_blue_x - segment_start + 1;
            if (segment_width >= MIN_BLUE_SEGMENT_WIDTH && segment_count < max_segments) {
                register_segment(segments, &segment_count, row, segment_start, last_blue_x);
            }
        }

        segment_start = -1;
        last_blue_x = -1;
        non_blue_gap = 0;
    }

    if (segment_start >= 0 && last_blue_x >= segment_start) {
        const int segment_width = last_blue_x - segment_start + 1;
        if (segment_width >= MIN_BLUE_SEGMENT_WIDTH && segment_count < max_segments) {
            register_segment(segments, &segment_count, row, segment_start, last_blue_x);
        }
    }

    return segment_count;
}

static void clean_blue_rows_in_roi(uint16_t *frame, int width, int height)
{
    const int rotated_width = height;
    const int rotated_height = width;
    int roi_top;
    int roi_bottom;

    compute_target_roi(rotated_height, &roi_top, &roi_bottom);

    for (int row = roi_top; row <= roi_bottom; ++row) {
        target_row_segment_t segments[MAX_SEGMENTS_PER_ROW];
        const int segment_count = find_blue_segments_in_row(frame,
                                                           width,
                                                           height,
                                                           row,
                                                           segments,
                                                           MAX_SEGMENTS_PER_ROW);

        for (int x = 0; x < rotated_width; ++x) {
            set_px_rotm90(frame, width, height, x, row, TARGET_COLOR_WHITE);
        }

        for (int i = 0; i < segment_count; ++i) {
            for (int x = segments[i].start_x; x <= segments[i].end_x; ++x) {
                set_px_rotm90(frame, width, height, x, row, TARGET_COLOR_BLUE);
            }
        }
    }
}

static void reset_candidate(target_candidate_t *candidate)
{
    candidate->active = false;
    candidate->min_x = 0;
    candidate->max_x = 0;
    candidate->min_y = 0;
    candidate->max_y = 0;
    candidate->last_row = 0;
    candidate->last_center_x = 0;
    candidate->segment_count = 0;
    candidate->area = 0;
    candidate->sum_center_x = 0;
    candidate->sum_center_y = 0;
}

static void start_candidate(target_candidate_t *candidate,
                            const target_row_segment_t *segment)
{
    candidate->active = true;
    candidate->min_x = segment->start_x;
    candidate->max_x = segment->end_x;
    candidate->min_y = segment->row;
    candidate->max_y = segment->row;
    candidate->last_row = segment->row;
    candidate->last_center_x = segment->center_x;
    candidate->segment_count = 1;
    candidate->area = segment->width;
    candidate->sum_center_x = segment->center_x;
    candidate->sum_center_y = segment->row;
}

static void add_segment_to_candidate(target_candidate_t *candidate,
                                     const target_row_segment_t *segment)
{
    if (segment->start_x < candidate->min_x) candidate->min_x = segment->start_x;
    if (segment->end_x > candidate->max_x) candidate->max_x = segment->end_x;
    if (segment->row < candidate->min_y) candidate->min_y = segment->row;
    if (segment->row > candidate->max_y) candidate->max_y = segment->row;

    candidate->last_row = segment->row;
    candidate->last_center_x = segment->center_x;
    candidate->segment_count++;
    candidate->area += segment->width;
    candidate->sum_center_x += segment->center_x;
    candidate->sum_center_y += segment->row;
}

static bool candidate_is_valid(const target_candidate_t *candidate)
{
    const int height = candidate->max_y - candidate->min_y + 1;

    if (!candidate->active) {
        return false;
    }

    if (height < MIN_TARGET_HEIGHT) {
        return false;
    }

    if (candidate->segment_count < MIN_TARGET_SEGMENTS) {
        return false;
    }

    if (candidate->area < MIN_TARGET_AREA) {
        return false;
    }

    return true;
}

static void publish_candidate(const target_candidate_t *candidate,
                              target_detection_t *targets,
                              int *target_count,
                              int max_targets)
{
    target_detection_t new_target;

    if (!candidate_is_valid(candidate)) {
        return;
    }

    new_target.found = true;
    new_target.min_x = candidate->min_x;
    new_target.max_x = candidate->max_x;
    new_target.min_y = candidate->min_y;
    new_target.max_y = candidate->max_y;
    new_target.width = candidate->max_x - candidate->min_x + 1;
    new_target.height = candidate->max_y - candidate->min_y + 1;
    new_target.segment_count = candidate->segment_count;
    new_target.center_x = (int)((candidate->sum_center_x + candidate->segment_count / 2) /
                                candidate->segment_count);
    new_target.center_y = (int)((candidate->sum_center_y + candidate->segment_count / 2) /
                                candidate->segment_count);

    for (int i = 0; i < *target_count; ++i) {
        const int overlap_w = overlap_len(new_target.min_x, new_target.max_x,
                                          targets[i].min_x, targets[i].max_x);
        const int overlap_h = overlap_len(new_target.min_y, new_target.max_y,
                                          targets[i].min_y, targets[i].max_y);
        const int overlap_area = overlap_w * overlap_h;
        const int new_area = new_target.width * new_target.height;
        const int old_area = targets[i].width * targets[i].height;
        const int smaller_area = min_int(new_area, old_area);

        if (smaller_area <= 0) {
            continue;
        }

        if ((overlap_area * TARGET_OVERLAP_REJECT_DEN) <
            (smaller_area * TARGET_OVERLAP_REJECT_NUM)) {
            continue;
        }

        if (new_target.segment_count > targets[i].segment_count ||
            (new_target.segment_count == targets[i].segment_count && new_area > old_area)) {
            targets[i] = new_target;
        }

        return;
    }

    if (*target_count >= max_targets) {
        return;
    }

    targets[*target_count] = new_target;
    (*target_count)++;
}

static int find_best_candidate(const target_candidate_t *candidates,
                               int candidate_count,
                               const target_row_segment_t *segment)
{
    int best_index = -1;
    int best_score = 0x7FFFFFFF;

    for (int i = 0; i < candidate_count; ++i) {
        int dx;
        int row_gap;
        int x_overlap;
        int score;

        if (!candidates[i].active) {
            continue;
        }

        row_gap = segment->row - candidates[i].last_row;
        if (row_gap < 0 || row_gap > MAX_TARGET_ROW_GAP) {
            continue;
        }

        dx = iabs_int(segment->center_x - candidates[i].last_center_x);
        x_overlap = overlap_len(segment->start_x,
                                segment->end_x,
                                candidates[i].min_x,
                                candidates[i].max_x);

        if (x_overlap <= 0 && dx > MAX_TARGET_CENTER_JUMP) {
            continue;
        }

        score = dx + (row_gap * 4) - (x_overlap * 2);

        if (score < best_score) {
            best_score = score;
            best_index = i;
        }
    }

    return best_index;
}

static int find_free_candidate(target_candidate_t *candidates, int candidate_count)
{
    for (int i = 0; i < candidate_count; ++i) {
        if (!candidates[i].active) {
            return i;
        }
    }

    return -1;
}

static void flush_old_candidates(target_candidate_t *candidates,
                                 int candidate_count,
                                 int current_row,
                                 target_detection_t *targets,
                                 int *target_count,
                                 int max_targets)
{
    for (int i = 0; i < candidate_count; ++i) {
        if (!candidates[i].active) {
            continue;
        }

        if ((current_row - candidates[i].last_row) <= MAX_TARGET_ROW_GAP) {
            continue;
        }

        publish_candidate(&candidates[i], targets, target_count, max_targets);
        reset_candidate(&candidates[i]);
    }
}

static void mark_segment_center(uint16_t *frame,
                                int width,
                                int height,
                                const target_row_segment_t *segment)
{
    set_px_rotm90(frame, width, height, segment->center_x, segment->row, TARGET_COLOR_GREEN);
}

int sort_targets(uint16_t *frame,
                 int width,
                 int height,
                 target_detection_t *targets,
                 int max_targets)
{
    target_candidate_t candidates[MAX_ACTIVE_TARGETS];
    int target_count = 0;
    const int rotated_height = width;
    int roi_top;
    int roi_bottom;

    if (!frame || !targets || width <= 0 || height <= 0 || max_targets <= 0) {
        return 0;
    }

    if (max_targets > FRAME_FIND_TARGET_MAX_TARGETS) {
        max_targets = FRAME_FIND_TARGET_MAX_TARGETS;
    }

    compute_target_roi(rotated_height, &roi_top, &roi_bottom);

    for (int i = 0; i < max_targets; ++i) {
        targets[i].found = false;
        reset_candidate(&candidates[i]);
    }

    for (int row = roi_top; row <= roi_bottom; ++row) {
        target_row_segment_t segments[MAX_SEGMENTS_PER_ROW];
        int segment_count = find_blue_segments_in_row(frame,
                                                      width,
                                                      height,
                                                      row,
                                                      segments,
                                                      MAX_SEGMENTS_PER_ROW);

        flush_old_candidates(candidates,
                             max_targets,
                             row,
                             targets,
                             &target_count,
                             max_targets);

        for (int i = 0; i < segment_count; ++i) {
            const int best_index = find_best_candidate(candidates, max_targets, &segments[i]);

            if (best_index >= 0) {
                add_segment_to_candidate(&candidates[best_index], &segments[i]);
            } else {
                const int free_index = find_free_candidate(candidates, max_targets);
                if (free_index >= 0) {
                    start_candidate(&candidates[free_index], &segments[i]);
                }
            }

            mark_segment_center(frame, width, height, &segments[i]);
        }
    }

    flush_old_candidates(candidates,
                         max_targets,
                         roi_bottom + MAX_TARGET_ROW_GAP + 1,
                         targets,
                         &target_count,
                         max_targets);

    return target_count;
}

static void draw_horizontal_line(uint16_t *frame,
                                 int width,
                                 int height,
                                 int x0,
                                 int x1,
                                 int y,
                                 uint16_t color)
{
    const int rotated_width = height;
    const int rotated_height = width;

    if (y < 0 || y >= rotated_height) {
        return;
    }

    if (x0 < 0) x0 = 0;
    if (x1 >= rotated_width) x1 = rotated_width - 1;

    for (int x = x0; x <= x1; ++x) {
        set_px_rotm90(frame, width, height, x, y, color);
    }
}

static void draw_vertical_line(uint16_t *frame,
                               int width,
                               int height,
                               int x,
                               int y0,
                               int y1,
                               uint16_t color)
{
    const int rotated_width = height;
    const int rotated_height = width;

    if (x < 0 || x >= rotated_width) {
        return;
    }

    if (y0 < 0) y0 = 0;
    if (y1 >= rotated_height) y1 = rotated_height - 1;

    for (int y = y0; y <= y1; ++y) {
        set_px_rotm90(frame, width, height, x, y, color);
    }
}

void draw_target(uint16_t *frame,
                 int width,
                 int height,
                 const target_detection_t *target,
                 uint16_t color)
{
    if (!frame || !target || !target->found) {
        return;
    }

    draw_horizontal_line(frame, width, height, target->min_x, target->max_x, target->min_y, color);
    draw_horizontal_line(frame, width, height, target->min_x, target->max_x, target->max_y, color);
    draw_vertical_line(frame, width, height, target->min_x, target->min_y, target->max_y, color);
    draw_vertical_line(frame, width, height, target->max_x, target->min_y, target->max_y, color);

    if (target->center_x >= 0 && target->center_x < height &&
        target->center_y >= 0 && target->center_y < width) {
        set_px_rotm90(frame, width, height, target->center_x, target->center_y, TARGET_COLOR_RED);
    }
}

void draw_target_roi(uint16_t *frame, int width, int height, uint16_t color)
{
    const int rotated_width = height;
    const int rotated_height = width;
    int roi_top;
    int roi_bottom;

    if (!frame || width <= 0 || height <= 0) {
        return;
    }

    compute_target_roi(rotated_height, &roi_top, &roi_bottom);

    draw_horizontal_line(frame, width, height, 0, rotated_width - 1, roi_top, color);
    draw_horizontal_line(frame, width, height, 0, rotated_width - 1, roi_bottom, color);
    draw_vertical_line(frame, width, height, 0, roi_top, roi_bottom, color);
    draw_vertical_line(frame, width, height, rotated_width - 1, roi_top, roi_bottom, color);
}
