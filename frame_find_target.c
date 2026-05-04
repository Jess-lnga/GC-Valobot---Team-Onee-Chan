// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jerome ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jerome ESSOLA ELANGA

#include "frame_find_target.h"

#include "frame_find_target_helpers.h"

static volatile int g_target_count = 0;
static target_detection_t g_targets[FRAME_FIND_TARGET_MAX_TARGETS];

static void clear_targets(void)
{
    g_target_count = 0;

    for (int i = 0; i < FRAME_FIND_TARGET_MAX_TARGETS; ++i) {
        g_targets[i].found = false;
        g_targets[i].center_x = -1;
        g_targets[i].center_y = -1;
        g_targets[i].min_x = -1;
        g_targets[i].max_x = -1;
        g_targets[i].min_y = -1;
        g_targets[i].max_y = -1;
        g_targets[i].width = 0;
        g_targets[i].height = 0;
        g_targets[i].segment_count = 0;
    }
}

uint16_t *find_targets(uint16_t *frame, int width, int height)
{
    if (!frame || width <= 0 || height <= 0) {
        clear_targets();
        return frame;
    }

    filter_blue_pxl(frame, width, height);

    g_target_count = sort_targets(frame,
                                  width,
                                  height,
                                  g_targets,
                                  FRAME_FIND_TARGET_MAX_TARGETS);

    for (int i = 0; i < g_target_count; ++i) {
        draw_target(frame, width, height, &g_targets[i], TARGET_COLOR_PURPLE);
    }

    draw_target_roi(frame, width, height, TARGET_COLOR_RED);

    return frame;
}

int get_target_count(void)
{
    return g_target_count;
}

bool is_target_found(void)
{
    return g_target_count > 0;
}

target_detection_t get_target(int index)
{
    target_detection_t empty_target = {
        false,
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        0,
        0,
        0
    };

    if (index < 0 || index >= g_target_count || index >= FRAME_FIND_TARGET_MAX_TARGETS) {
        return empty_target;
    }

    return g_targets[index];
}
