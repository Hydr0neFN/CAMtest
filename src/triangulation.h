#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "config.h"

/**
 * Estimate distance to a detected light source using stereo disparity.
 *
 * Geometry (parallel-axis stereo, cameras at same height):
 *   focal_px  = (FRAME_WIDTH / 2) / tan(STEREO_HFOV_DEG / 2 * PI/180)
 *   disparity = x_primary - x_secondary   (pixels)
 *   distance  = STEREO_BASELINE_M * focal_px / disparity   (metres)
 *
 * Sign convention:
 *   Place the secondary camera on the LEFT and the primary on the RIGHT.
 *   An object straight ahead projects to the same x in both cameras
 *   (disparity = 0, infinite distance).  An object in front-left projects
 *   further left in the secondary (smaller x_secondary), giving positive
 *   disparity and a finite distance.
 *
 * @param x_primary    Blob centroid X on the primary camera (pixels, 0=left)
 * @param x_secondary  Blob centroid X on the secondary camera (pixels, 0=left)
 * @return             Estimated distance in metres, or -1.0f if:
 *                       - disparity < STEREO_MIN_DISPARITY
 *                       - distance is outside the 0.5–200 m sanity range
 *
 * Reliable range with STEREO_BASELINE_M = 0.15 m at SVGA (800 px wide):
 *   ~3 m to ~50 m.  Below 3 m disparity is very large (high error);
 *   above 50 m disparity drops below 1 px and this function returns -1.
 *
 * Limitations / future work:
 *   - No lens distortion correction.
 *   - Assumes rectified stereo (cameras perfectly parallel and level).
 *     Toe-in mounting introduces systematic error.
 *   - Blob matching across cameras is done by the caller; this function
 *     only does the distance math.
 */
float triangulate_distance(uint16_t x_primary, uint16_t x_secondary);

#ifdef __cplusplus
}
#endif

#endif // TRIANGULATION_H
