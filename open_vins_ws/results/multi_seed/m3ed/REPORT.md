# OpenVINS results: m3ed (repeated runs)

16 sequences, 10 runs each, monocular camera + IMU. OpenVINS has no seed option: it fixes the OpenCV seed to 0, so the runs are repeats of the same configuration and differ only through multi-threading.

**Outcome of the 160 runs: 160 ok, 0 diverged, 0 no_init, 0 crashed.** *diverged*: Sim(3) ATE above 10 % of the path or Sim(3) scale outside 0.5–2; *no_init*: no trajectory; *crashed*: stopped before the end of the bag. Only the *ok* runs enter the statistics; the others are in `runs.csv` and each `<sequence>/metrics.csv`.

All errors are computed after a Sim(3) alignment to ground truth, the same way as for DPVO, so the two can be compared. OpenVINS estimates metric scale, so two tables are added: the ATE after an SE(3) alignment (no scale correction) and the Sim(3) scale (below 1: the estimate is too long). The last table is the processing rate: frames per second of estimator time (tracking + filter update, without reading the bag), with one run at a time on the machine. Every number below is built from the per-run **RMSE** of the metric.

Normalised errors: ATE trans as % of the ground truth path length; RPE trans as % of the segment length; RPE rot in deg per 100 m. ATE rot has no distance normalisation. RPE segments are measured along the ground truth path. Ground truth and estimate are the IMU pose.

## 1. Per sequence: statistics over the runs

### ATE trans Sim3

| sequence | path [m] | runs | outcome | mean [m] | std | median | min | max | mean [% of path] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 12.962 | 0.000 | 12.962 | 12.962 | 12.962 | 0.534 | 0.000 | 0.534 | 0.534 | 0.534 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 2.308 | 0.329 | 2.204 | 2.204 | 3.245 | 0.248 | 0.035 | 0.236 | 0.236 | 0.348 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 5.887 | 0.000 | 5.887 | 5.887 | 5.887 | 0.663 | 0.000 | 0.663 | 0.663 | 0.663 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 7.592 | 0.000 | 7.592 | 7.592 | 7.592 | 0.905 | 0.000 | 0.905 | 0.905 | 0.905 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 4.971 | 0.000 | 4.971 | 4.971 | 4.971 | 0.330 | 0.000 | 0.330 | 0.330 | 0.330 |
| car_urban_day_horse | 47 | 10 | 10 ok | 0.211 | 0.000 | 0.211 | 0.211 | 0.211 | 0.453 | 0.000 | 0.453 | 0.453 | 0.453 |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 3.126 | 0.000 | 3.126 | 3.126 | 3.126 | 0.224 | 0.000 | 0.224 | 0.224 | 0.224 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 1.689 | 0.000 | 1.689 | 1.689 | 1.689 | 0.977 | 0.000 | 0.977 | 0.977 | 0.977 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 12.445 | 0.000 | 12.445 | 12.445 | 12.445 | 0.908 | 0.000 | 0.908 | 0.908 | 0.908 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 8.346 | 0.000 | 8.346 | 8.346 | 8.346 | 0.528 | 0.000 | 0.528 | 0.528 | 0.528 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 5.453 | 0.000 | 5.453 | 5.453 | 5.453 | 0.364 | 0.000 | 0.364 | 0.364 | 0.364 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 4.424 | 0.000 | 4.424 | 4.424 | 4.424 | 0.314 | 0.000 | 0.314 | 0.314 | 0.314 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 0.600 | 0.000 | 0.600 | 0.600 | 0.600 | 0.344 | 0.000 | 0.344 | 0.344 | 0.344 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 1.113 | 0.000 | 1.113 | 1.113 | 1.113 | 0.632 | 0.000 | 0.632 | 0.632 | 0.632 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 5.770 | 0.000 | 5.770 | 5.770 | 5.770 | 0.421 | 0.000 | 0.421 | 0.421 | 0.421 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 4.391 | 0.000 | 4.391 | 4.391 | 4.391 | 0.279 | 0.000 | 0.279 | 0.279 | 0.279 |

### ATE rot

| sequence | path [m] | runs | outcome | mean [deg] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 4.579 | 0.000 | 4.579 | 4.579 | 4.579 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 4.045 | 0.125 | 4.005 | 4.005 | 4.400 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 4.421 | 0.000 | 4.421 | 4.421 | 4.421 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 23.102 | 0.000 | 23.102 | 23.102 | 23.102 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 1.197 | 0.000 | 1.197 | 1.197 | 1.197 |
| car_urban_day_horse | 47 | 10 | 10 ok | 2.006 | 0.000 | 2.006 | 2.006 | 2.006 |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 1.506 | 0.000 | 1.506 | 1.506 | 1.506 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 1.354 | 0.000 | 1.354 | 1.354 | 1.354 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 1.521 | 0.000 | 1.521 | 1.521 | 1.521 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 1.003 | 0.000 | 1.003 | 1.003 | 1.003 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 0.794 | 0.000 | 0.794 | 0.794 | 0.794 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 1.926 | 0.000 | 1.926 | 1.926 | 1.926 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 1.526 | 0.000 | 1.526 | 1.526 | 1.526 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 1.304 | 0.000 | 1.304 | 1.304 | 1.304 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 1.119 | 0.000 | 1.119 | 1.119 | 1.119 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 1.195 | 0.000 | 1.195 | 1.195 | 1.195 |

### RPE trans @10m

| sequence | path [m] | runs | outcome | mean [m] | std | median | min | max | mean [% of segment] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 1.005 | 0.000 | 1.005 | 1.005 | 1.005 | 10.051 | 0.000 | 10.051 | 10.051 | 10.051 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 0.547 | 0.029 | 0.538 | 0.538 | 0.629 | 5.467 | 0.288 | 5.376 | 5.376 | 6.287 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 0.485 | 0.000 | 0.485 | 0.485 | 0.485 | 4.854 | 0.000 | 4.854 | 4.854 | 4.854 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 1.083 | 0.000 | 1.083 | 1.083 | 1.083 | 10.827 | 0.000 | 10.827 | 10.827 | 10.827 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 1.119 | 0.000 | 1.119 | 1.119 | 1.119 | 11.195 | 0.000 | 11.195 | 11.195 | 11.195 |
| car_urban_day_horse | 47 | 10 | 10 ok | 0.408 | 0.000 | 0.408 | 0.408 | 0.408 | 4.083 | 0.000 | 4.083 | 4.083 | 4.083 |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 0.413 | 0.000 | 0.413 | 0.413 | 0.413 | 4.135 | 0.000 | 4.135 | 4.135 | 4.135 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 1.165 | 0.000 | 1.165 | 1.165 | 1.165 | 11.654 | 0.000 | 11.654 | 11.654 | 11.654 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 1.921 | 0.000 | 1.921 | 1.921 | 1.921 | 19.209 | 0.000 | 19.209 | 19.209 | 19.209 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 1.408 | 0.000 | 1.408 | 1.408 | 1.408 | 14.077 | 0.000 | 14.077 | 14.077 | 14.077 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 0.860 | 0.000 | 0.860 | 0.860 | 0.860 | 8.598 | 0.000 | 8.598 | 8.598 | 8.598 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 0.622 | 0.000 | 0.622 | 0.622 | 0.622 | 6.216 | 0.000 | 6.216 | 6.216 | 6.216 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 0.552 | 0.000 | 0.552 | 0.552 | 0.552 | 5.521 | 0.000 | 5.521 | 5.521 | 5.521 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 0.705 | 0.000 | 0.705 | 0.705 | 0.705 | 7.051 | 0.000 | 7.051 | 7.051 | 7.051 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 0.740 | 0.000 | 0.740 | 0.740 | 0.740 | 7.402 | 0.000 | 7.402 | 7.402 | 7.402 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 1.154 | 0.000 | 1.154 | 1.154 | 1.154 | 11.537 | 0.000 | 11.537 | 11.537 | 11.537 |

### RPE rot @10m

| sequence | path [m] | runs | outcome | mean [deg] | std | median | min | max | mean [deg/100m] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 0.289 | 0.000 | 0.289 | 0.289 | 0.289 | 2.886 | 0.000 | 2.886 | 2.886 | 2.886 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 0.539 | 0.000 | 0.540 | 0.539 | 0.540 | 5.394 | 0.003 | 5.395 | 5.385 | 5.396 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 0.324 | 0.000 | 0.324 | 0.324 | 0.324 | 3.236 | 0.000 | 3.236 | 3.236 | 3.236 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 0.199 | 0.000 | 0.199 | 0.199 | 0.199 | 1.991 | 0.000 | 1.991 | 1.991 | 1.991 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 0.322 | 0.000 | 0.322 | 0.322 | 0.322 | 3.221 | 0.000 | 3.221 | 3.221 | 3.221 |
| car_urban_day_horse | 47 | 10 | 10 ok | 0.300 | 0.000 | 0.300 | 0.300 | 0.300 | 2.999 | 0.000 | 2.999 | 2.999 | 2.999 |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 0.371 | 0.000 | 0.371 | 0.371 | 0.371 | 3.711 | 0.000 | 3.711 | 3.711 | 3.711 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 0.496 | 0.000 | 0.496 | 0.496 | 0.496 | 4.958 | 0.000 | 4.958 | 4.958 | 4.958 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 0.568 | 0.000 | 0.568 | 0.568 | 0.568 | 5.684 | 0.000 | 5.684 | 5.684 | 5.684 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 0.386 | 0.000 | 0.386 | 0.386 | 0.386 | 3.865 | 0.000 | 3.865 | 3.865 | 3.865 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 0.284 | 0.000 | 0.284 | 0.284 | 0.284 | 2.843 | 0.000 | 2.843 | 2.843 | 2.843 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 0.578 | 0.000 | 0.578 | 0.578 | 0.578 | 5.785 | 0.000 | 5.785 | 5.785 | 5.785 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 0.571 | 0.000 | 0.571 | 0.571 | 0.571 | 5.715 | 0.000 | 5.715 | 5.715 | 5.715 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 0.509 | 0.000 | 0.509 | 0.509 | 0.509 | 5.087 | 0.000 | 5.087 | 5.087 | 5.087 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 0.309 | 0.000 | 0.309 | 0.309 | 0.309 | 3.086 | 0.000 | 3.086 | 3.086 | 3.086 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 0.248 | 0.000 | 0.248 | 0.248 | 0.248 | 2.476 | 0.000 | 2.476 | 2.476 | 2.476 |

### RPE trans @50m

| sequence | path [m] | runs | outcome | mean [m] | std | median | min | max | mean [% of segment] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 3.561 | 0.000 | 3.561 | 3.561 | 3.561 | 7.123 | 0.000 | 7.123 | 7.123 | 7.123 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 1.519 | 0.089 | 1.491 | 1.491 | 1.773 | 3.038 | 0.179 | 2.981 | 2.981 | 3.546 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 1.593 | 0.000 | 1.593 | 1.593 | 1.593 | 3.186 | 0.000 | 3.186 | 3.186 | 3.186 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 3.810 | 0.000 | 3.810 | 3.810 | 3.810 | 7.620 | 0.000 | 7.620 | 7.620 | 7.620 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 2.415 | 0.000 | 2.415 | 2.415 | 2.415 | 4.830 | 0.000 | 4.830 | 4.830 | 4.830 |
| car_urban_day_horse | 47 | 10 | 10 ok | 0.532 | 0.000 | 0.532 | 0.532 | 0.532 | 1.065 | 0.000 | 1.065 | 1.065 | 1.065 |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 1.104 | 0.000 | 1.104 | 1.104 | 1.104 | 2.208 | 0.000 | 2.208 | 2.208 | 2.208 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 2.893 | 0.000 | 2.893 | 2.893 | 2.893 | 5.786 | 0.000 | 5.786 | 5.786 | 5.786 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 4.735 | 0.000 | 4.735 | 4.735 | 4.735 | 9.470 | 0.000 | 9.470 | 9.470 | 9.470 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 2.607 | 0.000 | 2.607 | 2.607 | 2.607 | 5.214 | 0.000 | 5.214 | 5.214 | 5.214 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 2.677 | 0.000 | 2.677 | 2.677 | 2.677 | 5.354 | 0.000 | 5.354 | 5.354 | 5.354 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 1.551 | 0.000 | 1.551 | 1.551 | 1.551 | 3.102 | 0.000 | 3.102 | 3.102 | 3.102 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 1.298 | 0.000 | 1.298 | 1.298 | 1.298 | 2.596 | 0.000 | 2.596 | 2.596 | 2.596 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 1.638 | 0.000 | 1.638 | 1.638 | 1.638 | 3.276 | 0.000 | 3.276 | 3.276 | 3.276 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 2.121 | 0.000 | 2.121 | 2.121 | 2.121 | 4.242 | 0.000 | 4.242 | 4.242 | 4.242 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 2.223 | 0.000 | 2.223 | 2.223 | 2.223 | 4.445 | 0.000 | 4.445 | 4.445 | 4.445 |

### RPE rot @50m

| sequence | path [m] | runs | outcome | mean [deg] | std | median | min | max | mean [deg/100m] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 0.704 | 0.000 | 0.704 | 0.704 | 0.704 | 1.408 | 0.000 | 1.408 | 1.408 | 1.408 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 0.766 | 0.015 | 0.761 | 0.761 | 0.808 | 1.531 | 0.030 | 1.522 | 1.522 | 1.617 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 0.783 | 0.000 | 0.783 | 0.783 | 0.783 | 1.566 | 0.000 | 1.566 | 1.566 | 1.566 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 0.625 | 0.000 | 0.625 | 0.625 | 0.625 | 1.251 | 0.000 | 1.251 | 1.251 | 1.251 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 0.854 | 0.000 | 0.854 | 0.854 | 0.854 | 1.707 | 0.000 | 1.707 | 1.707 | 1.707 |
| car_urban_day_horse | 47 | 10 | 10 ok | 0.643 | 0.000 | 0.643 | 0.643 | 0.643 | 1.286 | 0.000 | 1.286 | 1.286 | 1.286 |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 0.748 | 0.000 | 0.748 | 0.748 | 0.748 | 1.496 | 0.000 | 1.496 | 1.496 | 1.496 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 1.488 | 0.000 | 1.488 | 1.488 | 1.488 | 2.976 | 0.000 | 2.976 | 2.976 | 2.976 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 1.012 | 0.000 | 1.012 | 1.012 | 1.012 | 2.023 | 0.000 | 2.023 | 2.023 | 2.023 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 0.751 | 0.000 | 0.751 | 0.751 | 0.751 | 1.503 | 0.000 | 1.503 | 1.503 | 1.503 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 0.686 | 0.000 | 0.686 | 0.686 | 0.686 | 1.373 | 0.000 | 1.373 | 1.373 | 1.373 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 1.258 | 0.000 | 1.258 | 1.258 | 1.258 | 2.516 | 0.000 | 2.516 | 2.516 | 2.516 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 1.153 | 0.000 | 1.153 | 1.153 | 1.153 | 2.306 | 0.000 | 2.306 | 2.306 | 2.306 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 1.145 | 0.000 | 1.145 | 1.145 | 1.145 | 2.290 | 0.000 | 2.290 | 2.290 | 2.290 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 0.638 | 0.000 | 0.638 | 0.638 | 0.638 | 1.276 | 0.000 | 1.276 | 1.276 | 1.276 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 0.637 | 0.000 | 0.637 | 0.637 | 0.637 | 1.274 | 0.000 | 1.274 | 1.274 | 1.274 |

### RPE trans @100m

| sequence | path [m] | runs | outcome | mean [m] | std | median | min | max | mean [% of segment] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 4.948 | 0.000 | 4.948 | 4.948 | 4.948 | 4.948 | 0.000 | 4.948 | 4.948 | 4.948 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 2.217 | 0.090 | 2.188 | 2.188 | 2.472 | 2.217 | 0.090 | 2.188 | 2.188 | 2.472 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 2.876 | 0.000 | 2.876 | 2.876 | 2.876 | 2.876 | 0.000 | 2.876 | 2.876 | 2.876 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 6.477 | 0.000 | 6.477 | 6.477 | 6.477 | 6.477 | 0.000 | 6.477 | 6.477 | 6.477 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 3.406 | 0.000 | 3.406 | 3.406 | 3.406 | 3.406 | 0.000 | 3.406 | 3.406 | 3.406 |
| car_urban_day_horse | 47 | 10 | 10 ok | – | – | – | – | – | – | – | – | – | – |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 1.692 | 0.000 | 1.692 | 1.692 | 1.692 | 1.692 | 0.000 | 1.692 | 1.692 | 1.692 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 1.817 | 0.000 | 1.817 | 1.817 | 1.817 | 1.817 | 0.000 | 1.817 | 1.817 | 1.817 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 7.570 | 0.000 | 7.570 | 7.570 | 7.570 | 7.570 | 0.000 | 7.570 | 7.570 | 7.570 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 4.312 | 0.000 | 4.312 | 4.312 | 4.312 | 4.312 | 0.000 | 4.312 | 4.312 | 4.312 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 4.782 | 0.000 | 4.782 | 4.782 | 4.782 | 4.782 | 0.000 | 4.782 | 4.782 | 4.782 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 1.744 | 0.000 | 1.744 | 1.744 | 1.744 | 1.744 | 0.000 | 1.744 | 1.744 | 1.744 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 1.535 | 0.000 | 1.535 | 1.535 | 1.535 | 1.535 | 0.000 | 1.535 | 1.535 | 1.535 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 2.123 | 0.000 | 2.123 | 2.123 | 2.123 | 2.123 | 0.000 | 2.123 | 2.123 | 2.123 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 2.812 | 0.000 | 2.812 | 2.812 | 2.812 | 2.812 | 0.000 | 2.812 | 2.812 | 2.812 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 3.190 | 0.000 | 3.190 | 3.190 | 3.190 | 3.190 | 0.000 | 3.190 | 3.190 | 3.190 |

### RPE rot @100m

| sequence | path [m] | runs | outcome | mean [deg] | std | median | min | max | mean [deg/100m] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 1.117 | 0.000 | 1.117 | 1.117 | 1.117 | 1.117 | 0.000 | 1.117 | 1.117 | 1.117 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 1.006 | 0.032 | 0.996 | 0.996 | 1.096 | 1.006 | 0.032 | 0.996 | 0.996 | 1.096 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 1.160 | 0.000 | 1.160 | 1.160 | 1.160 | 1.160 | 0.000 | 1.160 | 1.160 | 1.160 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 1.154 | 0.000 | 1.154 | 1.154 | 1.154 | 1.154 | 0.000 | 1.154 | 1.154 | 1.154 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 0.976 | 0.000 | 0.976 | 0.976 | 0.976 | 0.976 | 0.000 | 0.976 | 0.976 | 0.976 |
| car_urban_day_horse | 47 | 10 | 10 ok | – | – | – | – | – | – | – | – | – | – |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 0.914 | 0.000 | 0.914 | 0.914 | 0.914 | 0.914 | 0.000 | 0.914 | 0.914 | 0.914 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 2.037 | 0.000 | 2.037 | 2.037 | 2.037 | 2.037 | 0.000 | 2.037 | 2.037 | 2.037 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 1.257 | 0.000 | 1.257 | 1.257 | 1.257 | 1.257 | 0.000 | 1.257 | 1.257 | 1.257 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 0.823 | 0.000 | 0.823 | 0.823 | 0.823 | 0.823 | 0.000 | 0.823 | 0.823 | 0.823 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 0.802 | 0.000 | 0.802 | 0.802 | 0.802 | 0.802 | 0.000 | 0.802 | 0.802 | 0.802 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 1.316 | 0.000 | 1.316 | 1.316 | 1.316 | 1.316 | 0.000 | 1.316 | 1.316 | 1.316 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 1.684 | 0.000 | 1.684 | 1.684 | 1.684 | 1.684 | 0.000 | 1.684 | 1.684 | 1.684 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 1.668 | 0.000 | 1.668 | 1.668 | 1.668 | 1.668 | 0.000 | 1.668 | 1.668 | 1.668 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 0.875 | 0.000 | 0.875 | 0.875 | 0.875 | 0.875 | 0.000 | 0.875 | 0.875 | 0.875 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 0.797 | 0.000 | 0.797 | 0.797 | 0.797 | 0.797 | 0.000 | 0.797 | 0.797 | 0.797 |

### ATE trans SE3

| sequence | path [m] | runs | outcome | mean [m] | std | median | min | max | mean [% of path] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 14.586 | 0.000 | 14.586 | 14.586 | 14.586 | 0.601 | 0.000 | 0.601 | 0.601 | 0.601 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 4.044 | 0.074 | 4.067 | 3.833 | 4.067 | 0.434 | 0.008 | 0.436 | 0.411 | 0.436 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 6.075 | 0.000 | 6.075 | 6.075 | 6.075 | 0.684 | 0.000 | 0.684 | 0.684 | 0.684 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 11.722 | 0.000 | 11.722 | 11.722 | 11.722 | 1.397 | 0.000 | 1.397 | 1.397 | 1.397 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 7.093 | 0.000 | 7.093 | 7.093 | 7.093 | 0.470 | 0.000 | 0.470 | 0.470 | 0.470 |
| car_urban_day_horse | 47 | 10 | 10 ok | 0.727 | 0.000 | 0.727 | 0.727 | 0.727 | 1.559 | 0.000 | 1.559 | 1.559 | 1.559 |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 3.280 | 0.000 | 3.280 | 3.280 | 3.280 | 0.235 | 0.000 | 0.235 | 0.235 | 0.235 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 1.954 | 0.000 | 1.954 | 1.954 | 1.954 | 1.130 | 0.000 | 1.130 | 1.130 | 1.130 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 15.153 | 0.000 | 15.153 | 15.153 | 15.153 | 1.106 | 0.000 | 1.106 | 1.106 | 1.106 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 10.333 | 0.000 | 10.333 | 10.333 | 10.333 | 0.654 | 0.000 | 0.654 | 0.654 | 0.654 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 7.496 | 0.000 | 7.496 | 7.496 | 7.496 | 0.500 | 0.000 | 0.500 | 0.500 | 0.500 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 4.902 | 0.000 | 4.902 | 4.902 | 4.902 | 0.348 | 0.000 | 0.348 | 0.348 | 0.348 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 1.207 | 0.000 | 1.207 | 1.207 | 1.207 | 0.693 | 0.000 | 0.693 | 0.693 | 0.693 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 1.279 | 0.000 | 1.279 | 1.279 | 1.279 | 0.726 | 0.000 | 0.726 | 0.726 | 0.726 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 7.286 | 0.000 | 7.286 | 7.286 | 7.286 | 0.531 | 0.000 | 0.531 | 0.531 | 0.531 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 7.296 | 0.000 | 7.296 | 7.296 | 7.296 | 0.464 | 0.000 | 0.464 | 0.464 | 0.464 |

### Sim3 scale

| sequence | path [m] | runs | outcome | mean [gt/est] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 0.986 | 0.000 | 0.986 | 0.986 | 0.986 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 0.976 | 0.003 | 0.975 | 0.975 | 0.985 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 0.989 | 0.000 | 0.989 | 0.989 | 0.989 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 1.036 | 0.000 | 1.036 | 1.036 | 1.036 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 0.957 | 0.000 | 0.957 | 0.957 | 0.957 |
| car_urban_day_horse | 47 | 10 | 10 ok | 0.956 | 0.000 | 0.956 | 0.956 | 0.956 |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 0.991 | 0.000 | 0.991 | 0.991 | 0.991 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 1.037 | 0.000 | 1.037 | 1.037 | 1.037 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 0.920 | 0.000 | 0.920 | 0.920 | 0.920 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 0.952 | 0.000 | 0.952 | 0.952 | 0.952 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 0.956 | 0.000 | 0.956 | 0.956 | 0.956 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 0.981 | 0.000 | 0.981 | 0.981 | 0.981 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 0.963 | 0.000 | 0.963 | 0.963 | 0.963 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 0.978 | 0.000 | 0.978 | 0.978 | 0.978 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 0.957 | 0.000 | 0.957 | 0.957 | 0.957 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 0.957 | 0.000 | 0.957 | 0.957 | 0.957 |

### Processing rate

| sequence | path [m] | runs | outcome | mean [fps] | std | median | min | max |
|---|---:|---:|---|---:|---:|---:|---:|---:|
| car_forest_into_ponds_long | 2427 | 10 | 10 ok | 45.129 | 0.340 | 45.235 | 44.600 | 45.620 |
| car_forest_into_ponds_short | 932 | 10 | 10 ok | 46.948 | 0.534 | 47.095 | 45.650 | 47.610 |
| car_forest_sand_1 | 888 | 10 | 10 ok | 46.350 | 0.731 | 46.595 | 44.460 | 46.930 |
| car_forest_tree_tunnel | 839 | 10 | 10 ok | 45.956 | 0.280 | 45.940 | 45.560 | 46.540 |
| car_urban_day_city_hall | 1508 | 10 | 10 ok | 50.821 | 0.608 | 50.910 | 49.920 | 51.700 |
| car_urban_day_horse | 47 | 10 | 10 ok | 50.563 | 0.578 | 50.500 | 49.770 | 51.370 |
| car_urban_day_penno_big_loop | 1398 | 10 | 10 ok | 45.396 | 0.563 | 45.555 | 44.060 | 46.110 |
| car_urban_day_penno_small_loop | 173 | 10 | 10 ok | 47.367 | 0.506 | 47.500 | 46.390 | 47.940 |
| car_urban_day_rittenhouse | 1370 | 10 | 10 ok | 46.850 | 0.235 | 46.820 | 46.460 | 47.200 |
| car_urban_day_ucity_small_loop | 1579 | 10 | 10 ok | 49.052 | 0.183 | 49.095 | 48.780 | 49.260 |
| car_urban_night_city_hall | 1499 | 10 | 10 ok | 51.555 | 0.671 | 51.690 | 50.440 | 52.490 |
| car_urban_night_penno_big_loop | 1410 | 10 | 10 ok | 44.201 | 0.568 | 44.390 | 43.380 | 44.910 |
| car_urban_night_penno_small_loop | 174 | 10 | 10 ok | 50.187 | 0.778 | 50.400 | 48.820 | 51.550 |
| car_urban_night_penno_small_loop_darker | 176 | 10 | 10 ok | 48.931 | 0.461 | 48.935 | 47.920 | 49.480 |
| car_urban_night_rittenhouse | 1371 | 10 | 10 ok | 48.303 | 0.536 | 48.260 | 47.410 | 49.350 |
| car_urban_night_ucity_small_loop | 1573 | 10 | 10 ok | 51.005 | 0.461 | 51.085 | 50.010 | 51.500 |

## 2. Per run: mean over the sequences

Run *k* is the *k*-th trial of every sequence, so one row is one complete pass over the dataset. The last rows are statistics over those passes. So that every row averages the same sequences, only the 16 of 16 sequences that are ok in every run are averaged. *ok* counts the ok sequences of that pass over all 16.

### Absolute

| run | ok | ATE trans Sim3 [m] | ATE rot [deg] | RPE trans @10m [m] | RPE rot @10m [deg] | RPE trans @50m [m] | RPE rot @50m [deg] | RPE trans @100m [m] | RPE rot @100m [deg] | ATE trans SE3 [m] | Sim3 scale [gt/est] | Processing rate [fps] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| run_00 | 16/16 | 5.139 | 3.309 | 0.892 | 0.393 | 2.283 | 0.871 | 3.450 | 1.178 | 6.514 | 0.975 | 48.094 |
| run_01 | 16/16 | 5.074 | 3.285 | 0.886 | 0.393 | 2.266 | 0.868 | 3.431 | 1.172 | 6.529 | 0.974 | 47.935 |
| run_02 | 16/16 | 5.074 | 3.285 | 0.886 | 0.393 | 2.266 | 0.868 | 3.431 | 1.172 | 6.529 | 0.974 | 48.378 |
| run_03 | 16/16 | 5.074 | 3.285 | 0.886 | 0.393 | 2.266 | 0.868 | 3.431 | 1.172 | 6.529 | 0.974 | 48.142 |
| run_04 | 16/16 | 5.074 | 3.285 | 0.886 | 0.393 | 2.266 | 0.868 | 3.431 | 1.172 | 6.529 | 0.974 | 47.873 |
| run_05 | 16/16 | 5.074 | 3.285 | 0.886 | 0.393 | 2.266 | 0.868 | 3.431 | 1.172 | 6.529 | 0.974 | 47.681 |
| run_06 | 16/16 | 5.074 | 3.285 | 0.886 | 0.393 | 2.266 | 0.868 | 3.431 | 1.172 | 6.529 | 0.974 | 47.918 |
| run_07 | 16/16 | 5.074 | 3.285 | 0.886 | 0.393 | 2.266 | 0.868 | 3.431 | 1.172 | 6.529 | 0.974 | 47.769 |
| run_08 | 16/16 | 5.074 | 3.285 | 0.886 | 0.393 | 2.266 | 0.868 | 3.431 | 1.172 | 6.529 | 0.974 | 48.224 |
| run_09 | 16/16 | 5.074 | 3.285 | 0.886 | 0.393 | 2.266 | 0.868 | 3.431 | 1.172 | 6.529 | 0.974 | 48.370 |
| **mean** | | **5.080** | **3.287** | **0.887** | **0.393** | **2.267** | **0.868** | **3.433** | **1.172** | **6.527** | **0.975** | **48.038** |
| **std** | | **0.021** | **0.008** | **0.002** | **0.000** | **0.006** | **0.001** | **0.006** | **0.002** | **0.005** | **0.000** | **0.242** |
| **median** | | **5.074** | **3.285** | **0.886** | **0.393** | **2.266** | **0.868** | **3.431** | **1.172** | **6.529** | **0.974** | **48.015** |
| **min** | | **5.074** | **3.285** | **0.886** | **0.393** | **2.266** | **0.868** | **3.431** | **1.172** | **6.514** | **0.974** | **47.681** |
| **max** | | **5.139** | **3.309** | **0.892** | **0.393** | **2.283** | **0.871** | **3.450** | **1.178** | **6.529** | **0.975** | **48.378** |

### Normalised

| run | ok | ATE trans Sim3 [% of path] | RPE trans @10m [% of segment] | RPE rot @10m [deg/100m] | RPE trans @50m [% of segment] | RPE rot @50m [deg/100m] | RPE trans @100m [% of segment] | RPE rot @100m [deg/100m] | ATE trans SE3 [% of path] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| run_00 | 16/16 | 0.514 | 8.919 | 3.933 | 4.566 | 1.742 | 3.450 | 1.178 | 0.719 |
| run_01 | 16/16 | 0.507 | 8.862 | 3.934 | 4.531 | 1.736 | 3.431 | 1.172 | 0.721 |
| run_02 | 16/16 | 0.507 | 8.862 | 3.934 | 4.531 | 1.736 | 3.431 | 1.172 | 0.721 |
| run_03 | 16/16 | 0.507 | 8.862 | 3.934 | 4.531 | 1.736 | 3.431 | 1.172 | 0.721 |
| run_04 | 16/16 | 0.507 | 8.862 | 3.934 | 4.531 | 1.736 | 3.431 | 1.172 | 0.721 |
| run_05 | 16/16 | 0.507 | 8.862 | 3.934 | 4.531 | 1.736 | 3.431 | 1.172 | 0.721 |
| run_06 | 16/16 | 0.507 | 8.862 | 3.934 | 4.531 | 1.736 | 3.431 | 1.172 | 0.721 |
| run_07 | 16/16 | 0.507 | 8.862 | 3.934 | 4.531 | 1.736 | 3.431 | 1.172 | 0.721 |
| run_08 | 16/16 | 0.507 | 8.862 | 3.934 | 4.531 | 1.736 | 3.431 | 1.172 | 0.721 |
| run_09 | 16/16 | 0.507 | 8.862 | 3.934 | 4.531 | 1.736 | 3.431 | 1.172 | 0.721 |
| **mean** | | **0.508** | **8.867** | **3.934** | **4.535** | **1.736** | **3.433** | **1.172** | **0.721** |
| **std** | | **0.002** | **0.018** | **0.000** | **0.011** | **0.002** | **0.006** | **0.002** | **0.000** |
| **median** | | **0.507** | **8.862** | **3.934** | **4.531** | **1.736** | **3.431** | **1.172** | **0.721** |
| **min** | | **0.507** | **8.862** | **3.933** | **4.531** | **1.736** | **3.431** | **1.172** | **0.719** |
| **max** | | **0.514** | **8.919** | **3.934** | **4.566** | **1.742** | **3.450** | **1.178** | **0.721** |

`summary_by_run.csv` also has the median over the sequences for every run.
