#include "triangulation.h"
#include <math.h>

// Focal length in pixels — computed once and cached.
// Depends only on FRAME_WIDTH and STEREO_HFOV_DEG, both compile-time constants.
static float s_focal_px = 0.0f;

static float get_focal_px(void)
{
    if (s_focal_px > 0.0f) return s_focal_px;
    float hfov_rad = (STEREO_HFOV_DEG * (float)M_PI) / 180.0f;
    s_focal_px = ((float)FRAME_WIDTH * 0.5f) / tanf(hfov_rad * 0.5f);
    return s_focal_px;
}

float triangulate_distance(uint16_t x_primary, uint16_t x_secondary)
{
    int disparity = (int)x_primary - (int)x_secondary;

    if (disparity < STEREO_MIN_DISPARITY) {
        return -1.0f;  // Object too far, or blob match is wrong
    }

    float distance = (STEREO_BASELINE_M * get_focal_px()) / (float)disparity;

    // Sanity bounds: closer than handlebars or beyond any useful headlight range
    if (distance < 0.5f || distance > 200.0f) {
        return -1.0f;
    }

    return distance;
}
