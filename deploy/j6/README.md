# Journey 6 YOLO artifact

`artifacts/best_road_j6e.hbm` is the quantized Journey 6E/M deployment form of
the repository's `best_road.pt` road-litter detector.

## Verified metadata

- Target BPU march: `nash-e`
- Toolchain: OpenExplorer 3.9.1, HBDK 4.11.11, `hb_compile` 3.5.16
- Quantization: calibrated PTQ INT8
- Runtime input: NV12 Y/UV, 640 x 640
- Output: `output0`, float32 `[1, 12, 8400]`; post-processing/NMS is external
- Classes: bottle, can, coffee_cup, crushed_can, food_can, paper_crumple,
  small_bottle, box
- Size: 4,405,536 bytes
- SHA-256: `c776c3f7d8417859858b8e027590ce783fd379a44d69ab0391a660a4b3f25254`

Verify the artifact after transfer:

```bash
sha256sum deploy/j6/artifacts/best_road_j6e.hbm
```

The HBM must be loaded with the BSP-compatible Journey 6 HBDNN/UCP runtime.
It cannot be passed to Ultralytics `YOLO()` on the ROS 2 computer. The existing
desktop detector therefore continues using `best_road.pt`; use this HBM only in
the J6 inference process and return decoded detections to ROS 2.

The compiler's 970.27 FPS / 1030.6 us values in `metrics/ptq_compile.yaml` are
estimates, not physical-board measurements. Do not present them as measured J6
performance until the HBM has been benchmarked on a board with the matching
runtime.
