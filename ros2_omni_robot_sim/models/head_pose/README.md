# YuNet face landmarks for approximate head orientation

Pinned OpenCV Zoo model `face_detection_yunet_2022mar.onnx`, commit
`c97242ce7f2a554e288b50eabd9f5df957e78801`, compatible model generation for
the FaceDetectorYN interface in ROS Humble's OpenCV 4.5.x. The newer 2023
model uses a different output layout and must not replace this file on 4.5.x.

Source: https://github.com/opencv/opencv_zoo/tree/c97242ce7f2a554e288b50eabd9f5df957e78801/models/face_detection_yunet

Original MIT license included in `LICENSE`; download URLs, sizes and SHA-256
checksums in `sources.json`. This model estimates faces and five landmarks.
Our `head_pose_estimation.py` fits a generic 3D face with OpenCV solvePnP,
then projects the head's forward vector into the map with capture-time TF.

This is not a trained eye-gaze estimator. Face size, occlusion, generic face
shape and landmark errors affect head yaw substantially. Minimum face size,
confidence, reprojection and depth-consistency checks reject weak estimates;
they are not a measured accuracy guarantee. No actor pose, authored group or
scenario gaze is supplied to the estimator or social-navigation node.

PnP conventions: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
