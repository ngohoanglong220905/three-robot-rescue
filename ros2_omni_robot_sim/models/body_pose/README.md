# Body pose baseline for ROS Humble / OpenCV DNN

Bundled `graph_opt.pb`: OpenPose MobileNet TensorFlow graph from
[quanhua92/human-pose-estimation-opencv](https://github.com/quanhua92/human-pose-estimation-opencv/tree/5f86f77242378d5343c6f8a589e40d21c8537b4e).
The author's example attributes the MobileNet graph to ildoonet/tf-pose-estimation.
Upstream Apache 2.0 license is included; pinned URL and SHA-256 in `sources.json`.

`body_pose_estimation.py` uses OpenCV DNN CPU directly, without TensorFlow,
PyTorch, CUDA, pip OpenCV or a NumPy upgrade. Each YOLO person crop is letterboxed,
then passed with RGB ordering, scale 1, mean 127.5. First 18 output channels are
COCO body heatmaps. Only confident in-crop peaks get capture-time RGB-D positions.

Shoulder and hip baselines estimate torso orientation. One baseline alone needs
face evidence to resolve the sign. Left/right landmark mistakes can still reverse
orientation; gates are not calibrated confidence probabilities. Missing joints
remain missing. Legs alone do not determine facing direction.

This is **not Part-HOE**. [Part-HOE / IROS 2024](https://arxiv.org/abs/2404.14139)
uses a learned orientation estimator for partial observations with confidence
estimation. Its [official implementation](https://github.com/zhaojieting/Part_HOE)
has a separate checkpoint and ViTPose-based dependencies. Neither its weights nor
its learned orientation head are installed here. The baseline provides image
keypoints and explicit unknown/partial states; it does not reproduce the paper's
partial-observation accuracy. Gazebo gesture recognition has not been measured.
