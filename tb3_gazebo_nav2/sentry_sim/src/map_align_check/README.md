# map_align_check (S2c)

Estimates the rigid 2-D offset between **Odin1's map frame** and the **field /
CAD frame** (the frame of the Nav2 map built from the field CAD) from
`/livox/lidar` + the CAD map. Use it to prove — and then correct — the situation
where Odin1's map frame does not coincide with the field, which would shift the
whole navigation stack.

## Sign convention

Estimated `(dx, dy, dyaw)` is the offset Odin1's map frame carries, i.e.

    p_map = R(dyaw) * p_field + (dx, dy)

This is exactly what `odin1_sim_bridge`'s `map_offset_x/y/yaw` parameters
inject, so an injected `(0.30, -0.20, 3 deg)` must come back as
`(0.30, -0.20, 3 deg)`. `--fix-yaml` also writes the inverse (`correction:`),
which is what you set the offset to in order to move the map back onto the
field.

## Usage

```bash
source /opt/ros/jazzy/setup.bash && source ~/sentry_ws/install/setup.bash
# one shot, robot parked (e.g. at the spawn pose)
ros2 run map_align_check map_align_check --sim-time --frames 5 \
  --out    ~/sentry_ws/results/map_align_check.json \
  --fix-yaml ~/sentry_ws/results/map_align_fix.yaml
# continuous while driving (optional)
ros2 run map_align_check map_align_check --sim-time --continuous --rate 0.5
# self test, no ROS needed
python3 ~/sentry_ws/src/map_align_check/test/test_estimator.py
```

Useful flags: `--map` (CAD yaml, default is the RMUL2027 3v3 CAD map),
`--frames`, `--z-min/--z-max` (default 0.10 / 1.20 m), `--range-max` (8 m),
`--max-points` (3000), `--xy-max/--xy-step` (±0.6 m in 2.5 cm steps),
`--yaw-max-deg/--yaw-step-deg` (±10° in 0.5° steps), `--expect=dx,dy,deg`
(prints the error), `--backend numpy` (no scipy), `--dump out.npz`.

Output: a one-line JSON prefixed `MAP_ALIGN_JSON` (also written to `--out`),
with `estimate`, `residual` (mean/median/p90 distance to the CAD surface and to
the nearest occupied cell), `confidence` (best vs second-best gap, i.e. how
unique the answer is), `points` and `timing` (capture / coarse / refine).

## How it works

1. `/livox/lidar` -> map frame with TF (`map <- livox_frame`), height band
   0.10..1.20 m in the map frame, horizontal range <= 8 m, thinned to 3000
   points.
2. CAD yaml/pgm -> occupied cells -> distance field to the free/occupied edge
   (the CAD surface). `scipy.ndimage.distance_transform_edt` when importable,
   otherwise a numpy 8-neighbour wavefront.
3. Coarse vectorised search over the whole ±0.6 m / ±10° grid (400 points),
   then a bounded pattern search on all points with bilinear field sampling.
4. Confidence = best vs second-best score gap over poses outside a 0.12 m /
   1.5° neighbourhood of the winner.

The objective is the mean distance from the observation to the nearest **CAD
surface** cell, not to the occupied area: a lidar return sits on the wall face,
one half cell outside the occupied cell centre, and scoring against the
occupied area gives the optimiser a free sideways slide that turned into a
systematic +0.57° yaw bias on this map (measured). The distance to the nearest
occupied cell is still reported as `residual.occupied_mean_m`.

Limitation: a long straight wall seen head-on constrains little along its
length. If the field around the robot is only one straight wall, treat a small
`confidence.gap_m` as "not enough structure" and drive somewhere with corners.