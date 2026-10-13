# Leader person detector model

YOLOv4-tiny, trained on COCO, used through OpenCV DNN on CPU. Class 0
(`person`) supplies people boxes; class 62 (`tv`) supplies candidate attention
targets. A separate YuNet + PnP stage estimates head direction. YOLO itself
does not estimate eye gaze or classify social activities.

- Upstream: https://github.com/AlexeyAB/darknet
- Weights: https://github.com/AlexeyAB/darknet/releases/download/yolov4/yolov4-tiny.weights
- Configuration pinned to commit `59596d7880f6504768df41d6daa586f5cb2b932f`.
- The original upstream `LICENSE` is included. `sources.json` records source
  URLs, SHA-256 checksums and sizes of the unmodified files.
- Files are installed with the ROS package; no model download is performed
  by a running launch. OpenCV / cv_bridge from Ubuntu / ROS Humble are used;
  PyTorch, CUDA and a pip replacement for NumPy are not required.

Input: centered square letterbox, padding 114, BGR converted to RGB by
`DetectionModel`, scale 1/255. Default input 416 × 416. OpenCV decodes the
Darknet Region outputs and performs confidence filtering / NMS. Boxes are
mapped back to the original image before depth lookup.

The model was trained on photographs, not on this Gazebo scene. Detection
quality on the supplied static human mesh has not been measured; partial
views, occlusion, small distant people and synthetic textures can cause misses
or false positives. Change `person_confidence` to tune the displayed results.
