# Per-camera keypoint tracking statistics from Basalt VIO.
# Published on /basalt_vio/keypoint_stats at VIO rate.
# ratio < 0.2 indicates degraded tracking; below this threshold the
# acceleration health check is likely to trip and trigger a VIO reset.

float32 ratio           # confirmed_left / tracked_left (primary health metric, 0.0–1.0)
uint32 tracked_left     # optical flow 2D features tracked in left camera this frame
uint32 tracked_right    # optical flow 2D features tracked in right camera this frame
uint32 confirmed_left   # VIO-confirmed 3D landmarks reprojected into left camera
uint32 confirmed_right  # VIO-confirmed 3D landmarks reprojected into right camera
