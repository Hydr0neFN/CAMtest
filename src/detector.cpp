#include "detector.h"
#include <string.h>
#include "esp_heap_caps.h"

// ---------------------------------------------------------------------------
// Union-Find for connected component labeling
// ---------------------------------------------------------------------------
// Max labels we can track. VGA worst-case is thousands, but in practice
// a thresholded night scene has very few bright regions.
#define MAX_LABELS 512

static uint16_t parent[MAX_LABELS];

static uint16_t uf_find(uint16_t x)
{
    while (parent[x] != x) {
        parent[x] = parent[parent[x]]; // path compression
        x = parent[x];
    }
    return x;
}

static void uf_union(uint16_t a, uint16_t b)
{
    a = uf_find(a);
    b = uf_find(b);
    if (a != b) {
        // Always merge higher label into lower
        if (a < b) parent[b] = a;
        else       parent[a] = b;
    }
}

// ---------------------------------------------------------------------------
// Per-label accumulator for computing blob stats in the second pass
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t sum_x;
    uint32_t sum_y;
    uint32_t pixel_count;
    uint32_t brightness_sum;
} label_acc_t;

// ---------------------------------------------------------------------------
// Blob detection — two-pass connected component labeling
// ---------------------------------------------------------------------------
void detect_blobs(const uint8_t *pixels, int width, int height,
                  detection_result_t *result)
{
    memset(result, 0, sizeof(*result));

    // Determine ROI bounds
    int y_start = ROI_Y_START;
    int y_end   = ROI_Y_END;
    if (y_end == 0 || y_end > height) y_end = height;
    if (y_start >= y_end) y_start = 0;

    int roi_height = y_end - y_start;
    int roi_pixels = width * roi_height;

    // Allocate label map in PSRAM (2 bytes per pixel)
    // For VGA: 640*480*2 = 614,400 bytes — fits easily in 4MB PSRAM
    uint16_t *labels = (uint16_t *)heap_caps_calloc(roi_pixels, sizeof(uint16_t),
                                                     MALLOC_CAP_SPIRAM);
    if (!labels) {
        // Fallback to regular malloc if PSRAM unavailable
        labels = (uint16_t *)calloc(roi_pixels, sizeof(uint16_t));
        if (!labels) return; // Out of memory
    }

    // Initialize union-find
    for (int i = 0; i < MAX_LABELS; i++) parent[i] = i;
    uint16_t next_label = 1; // Label 0 = background

    // Compute scene brightness while we scan
    uint64_t scene_sum = 0;

    // --- PASS 1: Assign labels and merge neighbors ---
    for (int ry = 0; ry < roi_height; ry++) {
        int frame_y = ry + y_start;
        for (int x = 0; x < width; x++) {
            int fi = frame_y * width + x;   // Index into frame
            int ri = ry * width + x;        // Index into ROI/label map

            uint8_t pix = pixels[fi];
            scene_sum += pix;

            if (pix < BRIGHTNESS_THRESHOLD) {
                labels[ri] = 0; // Background
                continue;
            }

            // Look at already-visited neighbors: left and above
            uint16_t left  = (x > 0)  ? labels[ri - 1]     : 0;
            uint16_t above = (ry > 0) ? labels[ri - width]  : 0;

            if (left == 0 && above == 0) {
                // New blob
                if (next_label < MAX_LABELS) {
                    labels[ri] = next_label++;
                } else {
                    labels[ri] = 0; // Too many labels, skip
                }
            } else if (left != 0 && above == 0) {
                labels[ri] = left;
            } else if (left == 0 && above != 0) {
                labels[ri] = above;
            } else {
                // Both neighbors labeled — take the smaller and merge
                labels[ri] = (left < above) ? left : above;
                if (left != above) {
                    uf_union(left, above);
                }
            }
        }
    }

    // Scene brightness
    uint32_t total_roi_pixels = (uint32_t)roi_pixels;
    result->scene_brightness = (uint32_t)(scene_sum / total_roi_pixels);

    // --- PASS 2: Resolve labels and accumulate stats ---
    // We only need accumulators for labels that actually exist
    int num_labels = (next_label < MAX_LABELS) ? next_label : MAX_LABELS;

    label_acc_t *accs = (label_acc_t *)heap_caps_calloc(num_labels, sizeof(label_acc_t),
                                                         MALLOC_CAP_SPIRAM);
    if (!accs) {
        accs = (label_acc_t *)calloc(num_labels, sizeof(label_acc_t));
        if (!accs) {
            heap_caps_free(labels);
            return;
        }
    }

    for (int ry = 0; ry < roi_height; ry++) {
        int frame_y = ry + y_start;
        for (int x = 0; x < width; x++) {
            int ri = ry * width + x;
            uint16_t lbl = labels[ri];
            if (lbl == 0) continue;

            uint16_t root = uf_find(lbl);
            uint8_t pix = pixels[frame_y * width + x];

            accs[root].sum_x          += x;
            accs[root].sum_y          += (frame_y);
            accs[root].pixel_count    += 1;
            accs[root].brightness_sum += pix;
        }
    }

    // Free label map — no longer needed
    heap_caps_free(labels);

    // --- Collect qualifying blobs sorted by size (largest first) ---
    result->blob_count = 0;

    for (int i = 1; i < num_labels; i++) {
        if (parent[i] != i) continue; // Not a root label
        label_acc_t *a = &accs[i];
        if (a->pixel_count < MIN_BLOB_PIXELS)  continue;
        if (a->pixel_count > MAX_BLOB_PIXELS)  continue;

        if (result->blob_count < MAX_BLOBS) {
            blob_t *b = &result->blobs[result->blob_count];
            b->cx             = (uint16_t)(a->sum_x / a->pixel_count);
            b->cy             = (uint16_t)(a->sum_y / a->pixel_count);
            b->pixel_count    = a->pixel_count;
            b->brightness_sum = a->brightness_sum;
            result->blob_count++;
        }
    }

    // Simple insertion sort by pixel_count descending (MAX_BLOBS is small)
    for (int i = 1; i < result->blob_count; i++) {
        blob_t tmp = result->blobs[i];
        int j = i - 1;
        while (j >= 0 && result->blobs[j].pixel_count < tmp.pixel_count) {
            result->blobs[j + 1] = result->blobs[j];
            j--;
        }
        result->blobs[j + 1] = tmp;
    }

    heap_caps_free(accs);
}

// ---------------------------------------------------------------------------
// Blob tracker — inter-frame classification with N-frame hysteresis
// ---------------------------------------------------------------------------

void tracker_reset(tracker_state_t *state)
{
    memset(state, 0, sizeof(*state));
}

void tracker_classify(tracker_state_t *state, detection_result_t *result)
{
    // matched[j] prevents two current blobs matching the same previous blob
    bool matched[MAX_BLOBS];
    memset(matched, 0, sizeof(matched));

    // match_map[i] = which previous-frame slot current blob i matched to.
    // -1 means unmatched (new blob or reflection-filtered).
    // We need this to remap the vote state arrays at the end, because the
    // state slots are reindexed to current-blob order after each frame.
    int match_map[MAX_BLOBS];
    for (int i = 0; i < MAX_BLOBS; i++) match_map[i] = -1;

    for (int i = 0; i < result->blob_count; i++) {
        blob_t *b = &result->blobs[i];

        // --- Own-headlight road-reflection filter ---
        // Large bright blobs in the bottom quarter of the frame are almost
        // certainly reflections of our own headlight off the road surface.
        // Classify immediately — no voting needed, geometry is conclusive.
        if ((int)b->cy > (FRAME_HEIGHT * 3 / 4) &&
            b->pixel_count > (uint32_t)(MAX_BLOB_PIXELS / 2)) {
            b->classification = BLOB_CLASS_STATIC_LIGHT;
            b->dx = 0;
            b->dy = 0;
            continue;
        }

        // --- Inter-frame motion matching ---
        if (state->count == 0) {
            // No previous frame — cannot classify yet
            b->classification = BLOB_CLASS_UNKNOWN;
            b->dx = 0;
            b->dy = 0;
            continue;
        }

        // Greedy nearest-neighbour match to previous-frame centroids
        int best_j    = -1;
        int best_dist = 0x7FFFFFFF;

        for (int j = 0; j < state->count; j++) {
            if (matched[j]) continue;
            int dx = (int)b->cx - (int)state->cx[j];
            int dy = (int)b->cy - (int)state->cy[j];
            int dist = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
            if (dist < best_dist) {
                best_dist = dist;
                best_j    = j;
            }
        }

        if (best_j < 0 || best_dist > TRACKER_MAX_MATCH_DIST) {
            // New blob — no history
            b->classification = BLOB_CLASS_UNKNOWN;
            b->dx = 0;
            b->dy = 0;
            continue;
        }

        matched[best_j] = true;
        match_map[i] = best_j;
        b->dx = (int16_t)((int)b->cx - (int)state->cx[best_j]);
        b->dy = (int16_t)((int)b->cy - (int)state->cy[best_j]);

        int motion = (b->dx < 0 ? -b->dx : b->dx) +
                     (b->dy < 0 ? -b->dy : b->dy);

        // TODO: subtract expected parallax drift from bike speed
        // (accelerometer / hall-effect wheel sensor) before classifying.

        blob_class_t raw_class;
        if (motion <= TRACKER_STATIC_THRESHOLD) {
            raw_class = BLOB_CLASS_STATIC_LIGHT;
        } else if (motion >= TRACKER_VEHICLE_THRESHOLD) {
            raw_class = BLOB_CLASS_VEHICLE;
        } else {
            raw_class = BLOB_CLASS_UNKNOWN;
        }

        // --- N-frame hysteresis ---
        // Only update confirmed_class after TRACKER_CONFIRM_FRAMES consecutive
        // frames agreeing on the same raw_class.
        if (raw_class == state->pending_class[best_j]) {
            if (state->vote_count[best_j] < 255) {
                state->vote_count[best_j]++;
            }
        } else {
            // New candidate — restart vote
            state->pending_class[best_j] = raw_class;
            state->vote_count[best_j]    = 1;
        }

        if (state->vote_count[best_j] >= TRACKER_CONFIRM_FRAMES) {
            state->confirmed_class[best_j] = state->pending_class[best_j];
        }

        b->classification = state->confirmed_class[best_j];
    }

    // --- Remap vote state from previous-frame slot indices to current-frame indices ---
    // The vote arrays (confirmed_class, pending_class, vote_count) were updated
    // at slot [best_j] (old index), but we're about to store centroids at slot [i]
    // (new index).  Without remapping, the next frame's matcher would find the
    // centroid at slot i but read stale vote data from a different slot.
    blob_class_t new_confirmed[MAX_BLOBS];
    blob_class_t new_pending[MAX_BLOBS];
    uint8_t      new_votes[MAX_BLOBS];
    memset(new_confirmed, 0, sizeof(new_confirmed));
    memset(new_pending,   0, sizeof(new_pending));
    memset(new_votes,     0, sizeof(new_votes));

    for (int i = 0; i < result->blob_count; i++) {
        int j = match_map[i];
        if (j >= 0) {
            new_confirmed[i] = state->confirmed_class[j];
            new_pending[i]   = state->pending_class[j];
            new_votes[i]     = state->vote_count[j];
        }
        // Unmatched blobs (j == -1) keep zeroed state — fresh start
    }
    memcpy(state->confirmed_class, new_confirmed, sizeof(new_confirmed));
    memcpy(state->pending_class,   new_pending,   sizeof(new_pending));
    memcpy(state->vote_count,      new_votes,     sizeof(new_votes));

    // Store current-frame centroids
    state->count = result->blob_count;
    for (int i = 0; i < result->blob_count; i++) {
        state->cx[i] = result->blobs[i].cx;
        state->cy[i] = result->blobs[i].cy;
    }

    // Reset when the scene goes dark — stale centroids from a now-gone light
    // would cause wrong matches on the next appearance.
    if (result->blob_count == 0) {
        tracker_reset(state);
    }
}
