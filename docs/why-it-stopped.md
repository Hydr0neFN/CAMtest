**English** · [繁體中文](why-it-stopped.zh-TW.md)

# Why it stopped — lessons

The pipeline *does* detect and triangulate bright blobs, but light-spot
detection proved too fragile in the real world. Road reflections, stray light,
and sensor noise generate too many false blobs to trust.

- **Light blobs are not unique.** Reflections off wet road, signs, parked cars,
  and other lights all read as the same bright blob the rider's lamp does.
- **Noise survives gating.** Even with size/brightness thresholds and 3-frame
  hysteresis, false blobs leaked through often enough to make a spotlight point
  at ghosts.
- **Stereo matching is brittle** when both cameras see multiple ambiguous blobs
  — disparity gets assigned to the wrong pair.

Conclusion: a single bright-spot cue is not a reliable signature for "oncoming
rider" in an uncontrolled outdoor scene. A more distinctive cue (modulated / IR
source, motion+shape, or an ML classifier) would be needed before the servo
spotlight stage is worth building.

Servo aiming was never wired up. The project stopped before actuator
integration. It is kept as a reference and as a record of the attempt.

See [noise-handling.md](noise-handling.md) for the filter list.
