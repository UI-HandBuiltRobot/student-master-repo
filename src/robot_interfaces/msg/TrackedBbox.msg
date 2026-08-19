# Width-normalized tracked bounding box with source annotation.
# Published by approach_action_server on WSKR/tracked_bbox once per image_callback.
# All spatial fields are normalized by the source frame WIDTH (not height).
# source is empty string when no bbox was produced this frame (no tracking).

float32 x_norm
float32 y_norm
float32 w_norm
float32 h_norm
string  source   # "yolo" | "csrt" | "aruco" | "" (no track this frame)
