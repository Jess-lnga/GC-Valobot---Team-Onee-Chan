// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jerome ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jerome ESSOLA ELANGA

#include "frame_find_target_helpers.h"

#define BLUE_MIN_B5                  10
#define BLUE_MIN_BRIGHTNESS          18
#define BLUE_RED_MARGIN               5
#define BLUE_GREEN_MARGIN             4

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

static int overlap_len(int a0, int a1, int b0, int b1)
{
    const int start = max_int(a0, b0);
    const int end = min_int(a1, b1);

    if (end < start) {
        return 0;
    }

    return end - start + 1;
}

static bool is_blue_pixel(uint16_t px)
{
    uint8_t r5, g6, b5;
    rgb565_to_components(px, &r5, &g6, &b5);

    const int g5 = (int)g6 >> 1;
    const int brightness = (int)r5 + g5 + (int)b5;

    if (brightness < BLUE_MIN_BRIGHTNESS) {
        return false;
    }

    if (b5 < BLUE_MIN_B5) {
        return false;
    }

    if (((int)b5 - (int)r5) < BLUE_RED_MARGIN) {
        return false;
    }

    if (((int)b5 - g5) < BLUE_GREEN_MARGIN) {
        return false;
    }

    return true;
}

void filter_blue_pxl(uint16_t *frame, int width, int height)
{
    const int pixel_count = width * height;

    for (int i = 0; i < pixel_count; ++i) {
        frame[i] = is_blue_pixel(frame[i]) ? TARGET_COLOR_BLUE : TARGET_COLOR_WHITE;
    }
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

    (void)height;

    if (!frame || !segments || row < 0 || row >= height || max_segments <= 0) {
        return 0;
    }

    if (max_segments > MAX_SEGMENTS_PER_ROW) {
        max_segments = MAX_SEGMENTS_PER_ROW;
    }

    for (int x = 0; x < width; ++x) {
        const bool is_blue = (frame[row * width + x] == TARGET_COLOR_BLUE);

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
                                const target_row_segment_t *segment)
{
    frame[segment->row * width + segment->center_x] = TARGET_COLOR_GREEN;
}

int sort_targets(uint16_t *frame,
                 int width,
                 int height,
                 target_detection_t *targets,
                 int max_targets)
{
    target_candidate_t candidates[MAX_ACTIVE_TARGETS];
    int target_count = 0;

    if (!frame || !targets || width <= 0 || height <= 0 || max_targets <= 0) {
        return 0;
    }

    if (max_targets > FRAME_FIND_TARGET_MAX_TARGETS) {
        max_targets = FRAME_FIND_TARGET_MAX_TARGETS;
    }

    for (int i = 0; i < max_targets; ++i) {
        targets[i].found = false;
        reset_candidate(&candidates[i]);
    }

    for (int row = 0; row < height; ++row) {
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

            mark_segment_center(frame, width, &segments[i]);
        }
    }

    flush_old_candidates(candidates,
                         max_targets,
                         height + MAX_TARGET_ROW_GAP + 1,
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
    if (y < 0 || y >= height) {
        return;
    }

    if (x0 < 0) x0 = 0;
    if (x1 >= width) x1 = width - 1;

    for (int x = x0; x <= x1; ++x) {
        frame[y * width + x] = color;
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
    if (x < 0 || x >= width) {
        return;
    }

    if (y0 < 0) y0 = 0;
    if (y1 >= height) y1 = height - 1;

    for (int y = y0; y <= y1; ++y) {
        frame[y * width + x] = color;
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

    if (target->center_x >= 0 && target->center_x < width &&
        target->center_y >= 0 && target->center_y < height) {
        frame[target->center_y * width + target->center_x] = TARGET_COLOR_RED;
    }
}
