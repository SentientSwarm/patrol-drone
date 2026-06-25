# patrol_interfaces/msg/CheckpointCapture
#
# The durable perception-capture contract (docset 04-perception, M6, PCAP-4).
# Emitted by patrol_perception on /patrol/checkpoint_capture and recorded into the
# MCAP bag by docset 05 (M7); later phases (3 YOLO, 4 relocalization, 6 anomaly)
# bind to this same shape. Field set is exactly the DoD-mandated set
# (header / checkpoint_id / pose / image / metadata) — no typed extension fields;
# `metadata` is the extension point. Design §4.2.3.

std_msgs/Header             header         # stamp + frame_id of the capture event
string                      checkpoint_id  # semantic id from 03's checkpoints.yaml (PCAP-2)
geometry_msgs/PoseStamped   pose           # capture pose, world/ENU, explicit frame_id (ADR-B)
string                      image_path     # path to the PNG/JPEG written by CaptureWriter (OQ-1 settled default)
diagnostic_msgs/KeyValue[]  metadata       # free-form key-value pairs (PCAP-6; ADR-C)
