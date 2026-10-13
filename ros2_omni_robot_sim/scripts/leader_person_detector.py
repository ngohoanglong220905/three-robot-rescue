#!/usr/bin/env python3
"""Detect COCO people in leader RGB, optionally locate visible surfaces with RGB-D.

No scenario annotations or Gazebo model poses are used. Inference has one worker
and consumes the latest RGB frame; camera and TF callbacks never wait for DNN.
"""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import math
import json
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from head_pose_estimation import estimate_heads
from body_pose_estimation import estimate_bodies, locate_joints, torso_orientation, visibility, PAIRS
from colorful_targets import colorful_regions, locate_colorful_regions


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def person_boxes(detector, image, confidence, nms, size):
    """Letterbox RGB-derived BGR data, then undo padding in pixel detections."""
    height, width = image.shape[:2]
    ratio = min(size / width, size / height)
    resized_w, resized_h = round(width * ratio), round(height * ratio)
    left, top = (size - resized_w) // 2, (size - resized_h) // 2
    padded = np.full((size, size, 3), 114, dtype=np.uint8)
    padded[top:top + resized_h, left:left + resized_w] = cv2.resize(
        image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    classes, scores, boxes = detector.detect(padded, confThreshold=confidence, nmsThreshold=nms)
    result, targets = [], []
    for class_id, score, box in zip(np.asarray(classes).reshape(-1),
                                    np.asarray(scores).reshape(-1), boxes):
        if int(class_id) not in (0, 62):  # COCO person / TV, zero-based IDs.
            continue
        x, y, w, h = (float(value) for value in box)
        x1, x2 = np.clip([(x - left) / ratio, (x + w - left) / ratio], 0, width - 1)
        y1, y2 = np.clip([(y - top) / ratio, (y + h - top) / ratio], 0, height - 1)
        if x2 - x1 >= 4 and y2 - y1 >= 8:
            output = result if int(class_id) == 0 else targets
            output.append((float(x1), float(y1), float(x2), float(y2), float(score)))
    return result, targets


def depth_position(depth, box, info, min_depth, max_depth):
    """Use central torso depth, excluding invalid/mixed foreground-background data.

    Returns a visible body-surface point, not an assumed body center or gaze.
    Input is aligned optical-axis depth in metres with undistorted intrinsics.
    """
    x1, y1, x2, y2, _ = box
    w, h = x2 - x1, y2 - y1
    left, right = int(x1 + 0.35 * w), int(math.ceil(x1 + 0.65 * w))
    top, bottom = int(y1 + 0.2 * h), int(math.ceil(y1 + 0.65 * h))
    patch = depth[top:bottom, left:right]
    valid = np.isfinite(patch) & (patch >= min_depth) & (patch <= max_depth)
    if valid.sum() < max(20, 0.3 * patch.size):
        return None
    samples = patch[valid]
    if np.percentile(samples, 90) - np.percentile(samples, 10) > 0.8:
        return None
    median = float(np.median(samples))
    mad = float(np.median(np.abs(samples - median)))
    if mad > 0.20:
        return None
    valid &= np.abs(patch - median) <= max(0.08, min(0.25, 2.5 * mad))
    if valid.sum() < 20:
        return None
    yy, xx = np.nonzero(valid)
    u, v = float(np.median(xx)) + left, float(np.median(yy)) + top
    z = float(np.median(patch[valid]))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=float)


def transform_point(point, transform):
    q = transform.rotation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quat)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError('Invalid camera transform quaternion')
    quat /= norm
    xyz, w = quat[:3], quat[3]
    rotated = point + 2.0 * np.cross(xyz, np.cross(xyz, point) + w * point)
    t = transform.translation
    return rotated + np.array([t.x, t.y, t.z])


class LeaderPersonDetector(Node):
    def __init__(self):
        super().__init__('leader_person_detector')
        model_dir = Path(get_package_share_directory('ros2_omni_robot_sim')) / 'models/person_detector'
        defaults = dict(
            model_config=str(model_dir / 'yolov4-tiny.cfg'),
            model_weights=str(model_dir / 'yolov4-tiny.weights'),
            confidence=0.35, nms_threshold=0.45, input_size=416,
            inference_rate=4.0, cpu_threads=2, max_frame_age=0.75,
            depth_sync_tolerance=0.06, min_depth=0.25, max_depth=7.5,
            map_frame='map', rgb_topic='/leader/camera/image_raw',
            depth_topic='/leader/camera/depth/image_raw',
            camera_info_topic='/leader/camera/camera_info',
            head_pose_enabled=True, face_confidence=0.85, minimum_face_size=24,
            face_model=str(model_dir.parent / 'head_pose/face_detection_yunet_2022mar.onnx'),
            body_pose_enabled=False, body_input_size=256, body_max_people=3,
            body_model=str(model_dir.parent / 'body_pose/graph_opt.pb'),
            colorful_targets_enabled=True,
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.options = {name: self.get_parameter(name).value for name in defaults}
        opts = self.options
        if not 0.0 < opts['confidence'] < 1.0 or not 0.0 < opts['nms_threshold'] < 1.0:
            raise ValueError('Confidence and NMS thresholds must be between 0 and 1')
        if not 0.0 < opts['face_confidence'] < 1.0 or opts['minimum_face_size'] < 12:
            raise ValueError('Face confidence must be in (0,1); minimum_face_size must be >= 12')
        if opts['input_size'] < 128 or opts['input_size'] % 32:
            raise ValueError('input_size must be a multiple of 32 and at least 128')
        if (opts['inference_rate'] <= 0 or opts['max_frame_age'] <= 0
                or opts['depth_sync_tolerance'] < 0
                or not 0 < opts['min_depth'] < opts['max_depth']):
            raise ValueError('Invalid inference rate, freshness limit or depth range')
        for name in ('model_config', 'model_weights'):
            if not Path(opts[name]).is_file():
                raise FileNotFoundError(f"Missing {name}: {opts[name]}. See README_SOCIAL_SCENE.md")
        cv2.setNumThreads(max(1, int(opts['cpu_threads'])))
        cv2.ocl.setUseOpenCL(False)
        net = cv2.dnn.readNetFromDarknet(opts['model_config'], opts['model_weights'])
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.detector = cv2.dnn_DetectionModel(net)
        self.detector.setInputParams(scale=1.0 / 255.0,
                                     size=(opts['input_size'], opts['input_size']),
                                     swapRB=True, crop=False)
        self.head_detector = None
        if opts['head_pose_enabled']:
            if not Path(opts['face_model']).is_file():
                raise FileNotFoundError(f"Missing face model: {opts['face_model']}")
            self.head_detector = cv2.FaceDetectorYN.create(
                opts['face_model'], '', (320, 320), opts['face_confidence'], 0.3, 1000)
        self.body_detector = None
        if opts['body_pose_enabled']:
            if opts['body_input_size'] < 128 or opts['body_input_size'] % 32 or opts['body_max_people'] < 1:
                raise ValueError('body_input_size must be a multiple of 32 >= 128; body_max_people >= 1')
            self.body_detector = cv2.dnn.readNetFromTensorflow(opts['body_model'])
            self.body_detector.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.body_detector.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0), node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rgb = None
        self.depths = deque(maxlen=16)
        self.info = None
        self.future = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='person_dnn')
        self.next_inference = 0.0
        self.last_image_wall = time.monotonic()
        self.last_status = 0.0
        self.last_clock = 0
        self.generation = 0
        self.frame_generation = 0
        self.stale_cleared = False
        self.pub_image = self.create_publisher(Image, '/leader/people/image', qos_profile_sensor_data)
        self.pub_2d = self.create_publisher(Detection2DArray, '/leader/people/detections_2d', 1)
        self.pub_poses = self.create_publisher(PoseArray, '/leader/people/poses', 1)
        self.pub_heads = self.create_publisher(PoseArray, '/leader/people/head_poses', 1)
        self.pub_targets = self.create_publisher(PoseArray, '/leader/people/attention_targets', 1)
        self.pub_color_targets = self.create_publisher(String, '/leader/people/color_targets', 1)
        self.pub_markers = self.create_publisher(MarkerArray, '/leader/people/markers', 1)
        self.pub_status = self.create_publisher(String, '/leader/people/status', 1)
        self.pub_frame_stamp = self.create_publisher(Header, '/leader/people/observation_stamp', 1)
        self.pub_bodies = self.create_publisher(String, '/leader/people/body_observations', 1)
        self.create_subscription(Image, opts['rgb_topic'], self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, opts['depth_topic'], self.depths.append, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, opts['camera_info_topic'], self.on_info, qos_profile_sensor_data)
        # Wall-clock timer also clears stale results if simulation /clock stops.
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"YOLOv4-tiny person detector ready: {opts['inference_rate']:.1f} Hz maximum, "
            f"CPU {opts['cpu_threads']} threads; waiting for leader RGB camera.")

    def on_rgb(self, msg):
        self.rgb = msg
        self.last_image_wall = time.monotonic()
        self.stale_cleared = False

    def on_info(self, msg):
        self.info = msg

    def status(self, text):
        now = time.monotonic()
        if now - self.last_status >= 1.0:
            self.pub_status.publish(String(data=text))
            self.get_logger().info(text, throttle_duration_sec=10.0)
            self.last_status = now

    def fresh(self, frame):
        age = (self.get_clock().now().nanoseconds - stamp_ns(frame.header)) / 1e9
        return -0.10 <= age <= self.options['max_frame_age']

    def empty(self):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.options['map_frame'])
        self.pub_poses.publish(PoseArray(header=header))
        self.pub_heads.publish(PoseArray(header=header))
        self.pub_targets.publish(PoseArray(header=header))
        self.publish_color_targets(header, [])
        rgb_header = self.rgb.header if self.rgb is not None else Header()
        self.pub_2d.publish(Detection2DArray(header=rgb_header))
        self.pub_markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def tick(self):
        wall_now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        if ros_now < self.last_clock:
            # Drop pre-reset frames and in-flight results after a Gazebo time reset.
            self.rgb = None
            self.depths.clear()
            self.info = None
            self.tf_buffer.clear()
            self.generation += 1
            self.empty()
        self.last_clock = ros_now
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                if (self.frame_generation == self.generation and self.fresh(result['rgb'])
                        and wall_now - result['started'] < 2.0):
                    self.publish_result(result)
                else:
                    self.empty()
                    self.status('Discarded stale inference result; awaiting a fresh camera frame')
            except Exception as exc:
                self.empty()
                self.get_logger().error(f'Person detection failed: {exc}', throttle_duration_sec=5.0)
            self.future = None
        if wall_now - self.last_image_wall > 2.0:
            if not self.stale_cleared:
                self.empty()
                self.stale_cleared = True
            self.status('Waiting for fresh RGB on ' + self.options['rgb_topic'])
            self.get_logger().warn('No fresh leader RGB images; check camera/bridge or paused Gazebo.',
                                   throttle_duration_sec=15.0)
            return
        if self.future is not None or self.rgb is None or wall_now < self.next_inference:
            return
        frame, self.rgb = self.rgb, None
        if not self.fresh(frame):
            self.empty()
            self.status('RGB timestamp is stale or ahead of ROS time; check /clock')
            return
        self.frame_generation = self.generation
        self.future = self.worker.submit(self.process, frame, self.info)
        self.next_inference = wall_now + 1.0 / self.options['inference_rate']

    def locate(self, frame, boxes):
        # Resolve depth and TF AFTER inference. Their callbacks can catch up to
        # the RGB timestamp while the worker runs, without blocking the executor.
        depth = min(self.depths, key=lambda item: abs(stamp_ns(item.header) - stamp_ns(frame.header)),
                    default=None)
        info = self.info
        reason = 'RGB only: awaiting aligned depth and camera_info'
        transform = None
        if (depth is not None and info is not None
                and abs(stamp_ns(depth.header) - stamp_ns(frame.header)) / 1e9
                <= self.options['depth_sync_tolerance']
                and frame.header.frame_id
                and frame.header.frame_id == depth.header.frame_id == info.header.frame_id
                and frame.width == depth.width == info.width
                and frame.height == depth.height == info.height
                and info.k[0] > 0 and info.k[4] > 0
                and all(abs(value) < 1e-6 for value in info.d)):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.options['map_frame'], frame.header.frame_id,
                    Time.from_msg(frame.header.stamp)).transform
                reason = 'RGB-D + capture-time TF'
            except TransformException:
                reason = 'RGB only: no map -> camera TF at image timestamp'
        else:
            depth = None
        depth_image = None
        if depth is not None and transform is not None:
            if depth.encoding in ('32FC1', '16UC1'):
                depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
                if depth.encoding == '16UC1':
                    depth_image = depth_image.astype(np.float32) * 0.001
            else:
                reason = f'RGB only: unsupported depth encoding {depth.encoding}'
        located = {}
        for index, box in enumerate(boxes):
            if depth_image is not None:
                optical = depth_position(depth_image, box, info,
                                         self.options['min_depth'], self.options['max_depth'])
                if optical is not None:
                    position = transform_point(optical, transform)
                    if np.all(np.isfinite(position)):
                        located[index] = (position, box[4], optical)
        return located, reason, transform, (depth_image, info)

    def process(self, rgb, info):
        started = time.monotonic()
        bgr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8').copy()
        boxes, targets = person_boxes(self.detector, bgr, self.options['confidence'],
                                      self.options['nms_threshold'], self.options['input_size'])
        colours = colorful_regions(bgr, boxes) if self.options['colorful_targets_enabled'] else []
        heads, head_error = {}, ''
        bodies, body_error = {}, ''
        if self.body_detector is not None:
            try:
                bodies = estimate_bodies(self.body_detector, bgr, boxes,
                                         self.options['body_input_size'], self.options['body_max_people'])
            except (cv2.error, ValueError) as exc:
                body_error = str(exc)
        if (self.head_detector is not None and info is not None
                and info.header.frame_id == rgb.header.frame_id):
            try:
                heads = estimate_heads(self.head_detector, bgr, boxes, info,
                                       self.options['minimum_face_size'])
            except cv2.error as exc:
                head_error = str(exc)
            # A face rendered inside a detected TV screen is not a nearby person.
            heads = {i: head for i, head in heads.items()
                     if not any(x1 <= (head[1][0] + head[1][2]) / 2 <= x2
                                and y1 <= (head[1][1] + head[1][3]) / 2 <= y2
                                for x1, y1, x2, y2, _ in targets)}
        ms = (time.monotonic() - started) * 1000.0
        return dict(rgb=rgb, bgr=bgr, boxes=boxes, targets=targets, heads=heads,
                    head_error=head_error, bodies=bodies, body_error=body_error,
                    colours=colours, started=started, ms=ms)

    def publish_color_targets(self, header, objects):
        self.pub_color_targets.publish(String(data=json.dumps(dict(
            version=1, frame_id=header.frame_id,
            stamp=dict(sec=header.stamp.sec, nanosec=header.stamp.nanosec),
            objects=objects), allow_nan=False)))

    def publish_result(self, result):
        header = result['rgb'].header
        if result['head_error']:
            self.get_logger().error('Head estimation unavailable: ' + result['head_error'],
                                    throttle_duration_sec=5.0)
        if result['body_error']:
            self.get_logger().error('Body pose unavailable: ' + result['body_error'], throttle_duration_sec=5.0)
        heads = list(result['heads'].items())
        count, target_count = len(result['boxes']), len(result['targets'])
        all_boxes = result['boxes'] + result['targets'] + [h[1][1] for h in heads]
        all_located, reason, transform, depth_context = self.locate(result['rgb'], all_boxes)
        located = {i: p for i, p in all_located.items() if i < count}
        bgr = result['bgr']
        detections = Detection2DArray(header=header)
        for index, (x1, y1, x2, y2, score) in enumerate(result['boxes']):
            color = (85, 225, 85) if index in located else (0, 190, 255)
            cv2.rectangle(bgr, (round(x1), round(y1)), (round(x2), round(y2)), color, 2)
            label = f'person {score:.2f} | ' + ('map OK' if index in located else '2D only')
            cv2.putText(bgr, label, (round(x1), max(18, round(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            detection = Detection2D(header=header)
            detection.bbox.center.position.x = (x1 + x2) / 2
            detection.bbox.center.position.y = (y1 + y2) / 2
            detection.bbox.size_x, detection.bbox.size_y = x2 - x1, y2 - y1
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'person'
            hypothesis.hypothesis.score = score
            detection.results.append(hypothesis)
            # id stays empty: single-frame detections do not imply identity tracking.
            detections.detections.append(detection)
        map_header = Header(stamp=header.stamp, frame_id=self.options['map_frame'])
        poses = PoseArray(header=map_header)
        head_poses = PoseArray(header=map_header)
        target_poses = PoseArray(header=map_header)
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        colours = locate_colorful_regions(result['colours'], *depth_context, transform, transform_point)
        self.publish_color_targets(map_header, colours)
        for index, obj in enumerate(colours):
            x1, y1, x2, y2 = map(round, obj['box'])
            cv2.rectangle(bgr, (x1, y1), (x2, y2), (220, 100, 240), 2)
            cv2.putText(bgr, f"colour target ({obj['hue_groups']} hues) | map OK",
                        (x1, max(40, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 100, 240), 1)
            label = Marker(header=map_header, ns='color_attention_targets', id=index,
                           type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = obj['position']
            label.pose.position.z += 0.20
            label.pose.orientation.w = 1.0
            label.scale.z = 0.15
            label.color.r, label.color.g, label.color.b, label.color.a = 0.95, 0.35, 0.85, 1.0
            label.text = 'camera: multicolour target (not TV classification)'
            label.lifetime = Duration(seconds=1.0).to_msg()
            markers.markers.append(label)
        for index, (xyz, score, _) in located.items():
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
            pose.orientation.w = 1.0  # Heading is unknown, not inferred from a 2D box.
            poses.poses.append(pose)
            dot = Marker(header=map_header, ns='camera_people', id=2 * index,
                         type=Marker.SPHERE, action=Marker.ADD, pose=pose)
            dot.scale.x = dot.scale.y = dot.scale.z = 0.35
            dot.color.r, dot.color.g, dot.color.b, dot.color.a = 0.15, 1.0, 0.25, 0.9
            dot.lifetime = Duration(seconds=1.0).to_msg()
            text = Marker(header=map_header, ns='camera_people', id=2 * index + 1,
                          type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            text.pose.position.x, text.pose.position.y = float(xyz[0]), float(xyz[1])
            text.pose.position.z = float(xyz[2]) + 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.20
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'camera: person {score:.2f}'
            text.lifetime = dot.lifetime
            markers.markers.extend((dot, text))
        head_angles = {}
        for index, (person_index, (forward, face_box, pnp_depth)) in enumerate(heads):
            measured = all_located.get(count + target_count + index)
            if measured is None or transform is None:
                continue
            xyz, _, optical = measured
            if abs(optical[2] - pnp_depth) > max(0.45, 0.45 * optical[2]):
                continue
            direction = transform_point(forward, transform) - transform_point(np.zeros(3), transform)
            if np.linalg.norm(direction[:2]) < 0.5 or not 0.5 < xyz[2] < 2.3:
                continue
            yaw = math.atan2(direction[1], direction[0])
            head_angles[person_index] = yaw
            head = Pose()
            head.position.x, head.position.y, head.position.z = map(float, xyz)
            head.orientation.z, head.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            head_poses.poses.append(head)
            if person_index not in located and not any(
                    math.hypot(p.position.x - xyz[0], p.position.y - xyz[1]) < 0.65 for p in poses.poses):
                # RGB-D localized head alone establishes a person position.
                # Do not discard its direction when YOLO misses an occluded body.
                person = Pose()
                person.position.x, person.position.y, person.position.z = map(float, xyz)
                person.orientation.w = 1.0
                poses.poses.append(person)
            arrow = Marker(header=map_header, ns='estimated_head_direction', id=1000 + index,
                           type=Marker.ARROW, action=Marker.ADD, pose=head)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.7, 0.035, 0.06
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.2, 0.85, 1.0, 1.0
            arrow.lifetime = Duration(seconds=1.0).to_msg()
            markers.markers.append(arrow)
            x1, y1, x2, y2, _ = face_box
            cv2.rectangle(bgr, (round(x1), round(y1)), (round(x2), round(y2)), (255, 220, 70), 1)
            cv2.putText(bgr, 'head estimate', (round(x1), max(35, round(y1) - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 220, 70), 1, cv2.LINE_AA)
        for index, (x1, y1, x2, y2, score) in enumerate(result['targets']):
            cv2.rectangle(bgr, (round(x1), round(y1)), (round(x2), round(y2)), (235, 120, 210), 2)
            cv2.putText(bgr, f'TV {score:.2f}', (round(x1), max(18, round(y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 120, 210), 1, cv2.LINE_AA)
            measured = all_located.get(count + index)
            if measured is not None:
                target = Pose()
                target.position.x, target.position.y, target.position.z = map(float, measured[0])
                target.orientation.w = 1.0
                target_poses.poses.append(target)
        bodies = []
        for index, points in result['bodies'].items():
            for a, b in PAIRS:
                if a in points and b in points:
                    cv2.line(bgr, tuple(round(v) for v in points[a][:2]),
                             tuple(round(v) for v in points[b][:2]), (245, 170, 40), 2)
            if index not in located:
                continue
            joints = locate_joints(points, *depth_context, transform, transform_point, located[index][2][2])
            heading, quality = torso_orientation(joints, head_angles.get(index))
            bodies.append(dict(position=list(map(float, located[index][0])), keypoints=joints,
                               heading=heading, heading_quality=quality, visibility=visibility(joints),
                               person_confidence=float(result['boxes'][index][4])))
        self.pub_bodies.publish(String(data=json.dumps(dict(
            version=1, frame_id=self.options['map_frame'],
            stamp=dict(sec=header.stamp.sec, nanosec=header.stamp.nanosec),
            people=bodies, source='yolo_crop_openpose_mobilenet_rgbd'), allow_nan=False)))
        cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 28), (25, 30, 35), -1)
        cv2.putText(bgr, f"People: {count} | Heads: {len(head_poses.poses)} | TV: {len(target_poses.poses)} | Color: {len(colours)} | {result['ms']:.0f} ms",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)
        image = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        image.header = header
        self.pub_image.publish(image)
        self.pub_2d.publish(detections)
        self.pub_poses.publish(poses)
        self.pub_heads.publish(head_poses)
        self.pub_targets.publish(target_poses)
        self.pub_markers.publish(markers)
        # Only completed, fresh RGB inference refreshes this heartbeat. Empty
        # watchdog messages must not authorize a blind observation sweep.
        self.pub_frame_stamp.publish(map_header)
        self.status(f"people={len(result['boxes'])}, located={len(poses.poses)}, "
                    f"heads={len(head_poses.poses)}, tv={len(target_poses.poses)}, "
                    f"bodies={len(bodies)}, "
                    f"color_targets={len(colours)}/{len(result['colours'])}, "
                    f"inference_ms={result['ms']:.0f}; {reason}"
                    + ('; HEAD MODEL ERROR' if result['head_error'] else '')
                    + ('; BODY MODEL ERROR' if result['body_error'] else ''))

    def destroy_node(self):
        self.timer.cancel()
        self.worker.shutdown(wait=True, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LeaderPersonDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
