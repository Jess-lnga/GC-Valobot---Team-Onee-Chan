#include "frame_analysis.h"

#include "frame_analysis_helpers.h"

static volatile int g_line_pos = 60;
static volatile int g_line_found = 0;
static volatile int g_line_measurement_valid = 0;
static volatile int g_line_lost_frames = 0;
static volatile int g_last_seen_side = 0;

uint16_t *find_line_pos(uint16_t *frame, int width, int height)
{
    if (!frame || width <= 0 || height <= 0) {
        g_line_found = 0;
        g_line_measurement_valid = 0;
        g_line_lost_frames++;
        return frame;
    }

    filter_black_pxl(frame, width, height);
    find_black_segments(frame, width, height);

    line_control_point_t control_point = sort_line(frame, width, height);
    if (!control_point.found) {
        g_line_found = 0;
        g_line_measurement_valid = 0;
        g_line_lost_frames++;
        return frame;
    }

    g_line_pos = control_point.center_x;
    g_line_found = 1;
    g_line_measurement_valid = 1;
    g_line_lost_frames = 0;

    if (g_line_pos < (height / 2)) {
        g_last_seen_side = -1;
    } else if (g_line_pos > (height / 2)) {
        g_last_seen_side = 1;
    }

    return frame;
}

int get_line_pos(void)
{
    return g_line_pos;
}

int is_line_found(void)
{
    return g_line_found;
}

int is_line_measurement_valid(void)
{
    return g_line_measurement_valid;
}

int get_line_lost_frames(void)
{
    return g_line_lost_frames;
}

int get_last_seen_side(void)
{
    return g_last_seen_side;
}
