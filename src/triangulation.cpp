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

float triangulate_distance(uint16_t px, uint16_t py,
                           uint16_t sx, uint16_t sy)
{
    // X-component sanity: secondary (LEFT) must see blob further right
    // than primary (RIGHT) for a valid forward object.
    int dx = (int)sx - (int)px;
    if (dx < STEREO_MIN_DISPARITY) {
        return -1.0f;
    }

    // 2D disparity — works when the bike leans and the baseline rotates.
    // When upright, dy ≈ 0 and this reduces to the standard X-only formula.
    int dy = (int)sy - (int)py;
    float disparity = sqrtf((float)(dx * dx + dy * dy));

    if (disparity < (float)STEREO_MIN_DISPARITY) {
        return -1.0f;
    }

    float distance = (STEREO_BASELINE_M * get_focal_px()) / disparity;

    // Sanity bounds
    if (distance < 0.5f || distance > 80.0f) {
        return -1.0f;
    }

    return distance;
}
