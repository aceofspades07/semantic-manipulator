"""YOLO model validation script."""

from ultralytics import YOLO

model = YOLO("yolo26n-seg.pt")

metrics = model.val()
metrics.box.map
metrics.box.map50
metrics.box.map75
metrics.box.maps
metrics.seg.map
metrics.seg.map50
metrics.seg.map75
metrics.seg.maps