**English** · [繁體中文](noise-handling.zh-TW.md)

# Noise handling — what was tried

Bright-spot detection drowns in false positives outdoors, so the pipeline
stacks several filters. They run in order; a blob has to survive all of them to
be reported as a real target.

1. **Brightness threshold** (`detector.cpp`) — only pixels ≥ 200/255 count.
   This kills the dim background.
2. **Size gating** — blobs smaller than 16 px are noise; larger than 70000 px are
   whole-frame washout. Both dropped.
3. **8-connectivity CCL + merge** — connected-component labeling groups bright
   pixels; centroids within 30 px are merged (a phone flashlight's two LED dies
   would otherwise read as two targets).
4. **Sensor-edge rejection** — blobs in the first/last few rows (`cy < 3` or
   `cy > height-4`) are discarded; `vflip`/`hmirror` produce bright-line
   artifacts there.
5. **Own-headlight reflection filter** (`tracker_classify`) — a large bright blob
   in the bottom quarter of the frame is almost certainly our own headlight
   bouncing off the road. Geometry is conclusive, so it's force-classified as
   `STATIC_LIGHT` with no voting.
6. **Inter-frame hysteresis** — a blob is matched to the previous frame by
   nearest centroid and must agree on the same class for **3 consecutive
   frames** (`TRACKER_CONFIRM_FRAMES`) before that class is trusted.
   Single-frame flickers never get confirmed.
7. **Cross-camera X/Y match — the main filter** (`main.cpp`, primary loop). A
   primary-camera blob is only accepted if a secondary-camera blob exists with:
   - **positive X-disparity** — the left (secondary) camera must see the blob
     *further right* than the right (primary) camera
     (`dx ≥ STEREO_MIN_DISPARITY`), which is geometrically required for a real
     object in front; and
   - **close 2D proximity** — Manhattan distance `dx + |dy|` under ~200 px, so
     the two views are looking at the same thing.

   A blob seen by only one camera, or with the wrong disparity sign, is
   rejected outright. This single check removes most single-camera reflections.
8. **Triangulation sanity bounds** (`triangulation.cpp`) — the computed distance
   must fall in 0.5–80 m and disparity must be ≥ 1 px, else the match is voided.
9. **Scene-dark reset** — when no blobs are present the tracker clears its
   state, so stale centroids from a light that has left the frame can't
   mismatch later.

**Still leaks through.** The cross-camera X/Y match is the strongest layer, but
some false targets survive it: when both cameras genuinely see the same
spurious bright spot — a wet-road reflection, a sign, a second light source —
it satisfies positive disparity *and* proximity and gets triangulated as a real
target. No amount of single-cue filtering separates "rider's lamp" from "any
other bright thing both cameras can see," which is the core reason the project
was shelved.

See [why-it-stopped.md](why-it-stopped.md) for the conclusion.
