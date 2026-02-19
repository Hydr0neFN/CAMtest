#ifndef DETECTOR_H
#define DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "config.h"

// ---------------------------------------------------------------------------
// Blob classification
// ---------------------------------------------------------------------------
typedef enum {
    BLOB_CLASS_UNKNOWN      = 0,   // Not yet confirmed (waiting for N frames)
    BLOB_CLASS_STATIC_LIGHT = 1,   // Streetlamp or stationary reflection
    BLOB_CLASS_VEHICLE      = 2,   // Oncoming vehicle headlight
} blob_class_t;

// ---------------------------------------------------------------------------
// Blob descriptor
// ---------------------------------------------------------------------------
typedef struct {
    uint16_t     cx;              // Centroid X
    uint16_t     cy;              // Centroid Y
    uint32_t     pixel_count;     // Number of bright pixels in this blob
    uint32_t     brightness_sum;  // Sum of pixel values (for computing average)
    blob_class_t classification;  // Filled in by tracker_classify()
    int16_t      dx;              // Inter-frame centroid delta X (set by tracker)
    int16_t      dy;              // Inter-frame centroid delta Y (set by tracker)
} blob_t;

// ---------------------------------------------------------------------------
// Detection result
// ---------------------------------------------------------------------------
typedef struct {
    blob_t   blobs[MAX_BLOBS];
    int      blob_count;        // How many blobs found (up to MAX_BLOBS)
    uint32_t scene_brightness;  // Average brightness of entire frame (0-255)
} detection_result_t;

// ---------------------------------------------------------------------------
// Tracker state — persists between frames
// Holds previous-frame centroids plus per-slot hysteresis vote counters.
// Zero-initialise on first use; call tracker_reset() to clear.
// ---------------------------------------------------------------------------
typedef struct {
    uint16_t     cx[MAX_BLOBS];             // Previous-frame centroid X
    uint16_t     cy[MAX_BLOBS];             // Previous-frame centroid Y
    blob_class_t confirmed_class[MAX_BLOBS];// Last classification that reached TRACKER_CONFIRM_FRAMES
    blob_class_t pending_class[MAX_BLOBS];  // Classification being voted on right now
    uint8_t      vote_count[MAX_BLOBS];     // Consecutive frames agreeing on pending_class
    int          count;                     // Number of valid slots from last frame
} tracker_state_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

/**
 * Detect bright blobs in a grayscale frame.
 * classification / dx / dy fields in result are left zeroed (BLOB_CLASS_UNKNOWN).
 * Call tracker_classify() afterward to fill them in.
 *
 * @param pixels  Raw grayscale pixel data (row-major, 1 byte per pixel)
 * @param width   Frame width  (use fb->width, not FRAME_WIDTH macro)
 * @param height  Frame height (use fb->height, not FRAME_HEIGHT macro)
 * @param result  Output: detected blobs and scene info
 */
void detect_blobs(const uint8_t *pixels, int width, int height,
                  detection_result_t *result);

/**
 * Match current blobs to previous frame, compute dx/dy, apply N-frame
 * hysteresis, and set the classification field on each blob.
 * Updates state with current-frame centroids ready for the next call.
 *
 * @param state   Persistent tracker state (caller owns; zero-init before first call)
 * @param result  Detection result to classify in-place
 */
void tracker_classify(tracker_state_t *state, detection_result_t *result);

/**
 * Reset the tracker (call on camera init, or when the scene changes drastically).
 */
void tracker_reset(tracker_state_t *state);

// ---------------------------------------------------------------------------
// Inline helpers
// ---------------------------------------------------------------------------

/** Average brightness of a blob (0-255). */
static inline uint8_t blob_avg_brightness(const blob_t *b)
{
    if (b->pixel_count == 0) return 0;
    return (uint8_t)(b->brightness_sum / b->pixel_count);
}

/** Human-readable classification string for serial output. */
static inline const char *blob_class_str(blob_class_t c)
{
    switch (c) {
        case BLOB_CLASS_STATIC_LIGHT: return "STATIC_LIGHT";
        case BLOB_CLASS_VEHICLE:      return "VEHICLE";
        default:                      return "UNKNOWN";
    }
}

#ifdef __cplusplus
}
#endif

#endif // DETECTOR_H
