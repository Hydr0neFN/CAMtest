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
 * Uses 2D disparity (Euclidean distance between blob centroids across cameras)
 * so that the measurement remains valid when the bike leans into turns and the
 * stereo baseline rotates away from horizontal.
 *
 * Geometry:
 *   focal_px  = (FRAME_WIDTH / 2) / tan(STEREO_HFOV_DEG / 2 * PI/180)
 *   disparity = sqrt((x2-x1)^2 + (y2-y1)^2)   (2D pixel distance)
 *   distance  = STEREO_BASELINE_M * focal_px / disparity   (metres)
 *
 * Sign convention:
 *   Secondary camera is on the LEFT, primary on the RIGHT.
 *   For objects ahead, the secondary sees the blob shifted right (higher cx)
 *   relative to the primary.  We still require positive X-disparity as a
 *   sanity check (x_secondary > x_primary), but the distance is computed
 *   from the full 2D disparity magnitude.
 *
 * @param px  Primary blob centroid X (pixels)
 * @param py  Primary blob centroid Y (pixels)
 * @param sx  Secondary blob centroid X (pixels)
 * @param sy  Secondary blob centroid Y (pixels)
 * @return    Estimated distance in metres, or -1.0f if invalid
 */
float triangulate_distance(uint16_t px, uint16_t py,
                           uint16_t sx, uint16_t sy);

#ifdef __cplusplus
}
#endif

#endif // TRIANGULATION_H
