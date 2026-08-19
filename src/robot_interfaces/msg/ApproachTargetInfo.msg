# Latched advertisement of the current approach target. Published by
# wskr_approach_action once on goal acceptance, again on goal end (with
# active=false), and on track-id handoffs during TOY approaches.
#
# Consumers (e.g. the dashboard) can filter the YOLO detection stream
# down to just this (class_name, track_id) pair while active is true.
uint8 TARGET_TOY=0
uint8 TARGET_BOX=1

string class_name    # YOLO class for TOY; empty or "box" for BOX/ArUco.
int32  track_id      # ByteTrack id for TOY, ArUco marker id for BOX, -1 if unknown.
uint8  target_type   # TARGET_TOY or TARGET_BOX.
bool   active        # false when no approach goal is in flight.
