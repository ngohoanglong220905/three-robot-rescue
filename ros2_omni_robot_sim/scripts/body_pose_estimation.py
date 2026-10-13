"""OpenPose MobileNet keypoints and RGB-D torso orientation (not Part-HOE).

Run one pose inference per YOLO crop. Never take heatmap maxima over a scene
containing multiple people, which can join shoulders from different people.
"""
import math
import cv2
import numpy as np


PAIRS = [(1, 2), (2, 3), (3, 4), (1, 5), (5, 6), (6, 7),
         (1, 8), (8, 9), (9, 10), (1, 11), (11, 12), (12, 13),
         (1, 0), (0, 14), (14, 16), (0, 15), (15, 17)]


def estimate_bodies(net, image, boxes, size=256, maximum=3, threshold=0.25):
    result = {}
    # Prefer larger, clearer bodies; bound CPU time when a crowd fills the image.
    order = sorted(range(len(boxes)), key=lambda i: (boxes[i][2] - boxes[i][0]) *
                   (boxes[i][3] - boxes[i][1]), reverse=True)[:maximum]
    for index in order:
        x1, y1, x2, y2, _ = boxes[index]
        if y2 - y1 < 48 or x2 - x1 < 16:
            continue
        padx, pady = 0.08 * (x2 - x1), 0.04 * (y2 - y1)
        left, right = max(0, int(x1 - padx)), min(image.shape[1], int(x2 + padx + 1))
        top, bottom = max(0, int(y1 - pady)), min(image.shape[0], int(y2 + pady + 1))
        crop = image[top:bottom, left:right]
        ratio = min(size / crop.shape[1], size / crop.shape[0])
        width, height = round(crop.shape[1] * ratio), round(crop.shape[0] * ratio)
        dx, dy = (size - width) // 2, (size - height) // 2
        padded = np.full((size, size, 3), 127, dtype=np.uint8)
        padded[dy:dy + height, dx:dx + width] = cv2.resize(crop, (width, height))
        blob = cv2.dnn.blobFromImage(padded, 1.0, (size, size), (127.5, 127.5, 127.5), swapRB=True)
        net.setInput(blob)
        heatmaps = net.forward()
        if heatmaps.ndim != 4 or heatmaps.shape[1] < 18:
            raise ValueError('Body model must output NCHW COCO-18 heatmaps')
        points = {}
        for joint, heatmap in enumerate(heatmaps[0, :18]):
            _, confidence, _, peak = cv2.minMaxLoc(heatmap)
            if not math.isfinite(confidence) or confidence < threshold:
                continue
            px = ((peak[0] + 0.5) * size / heatmap.shape[1] - dx) / ratio + left
            py = ((peak[1] + 0.5) * size / heatmap.shape[0] - dy) / ratio + top
            if left <= px < right and top <= py < bottom:
                points[joint] = (float(px), float(py), min(1.0, float(confidence)))
        result[index] = points
    return result


def locate_joints(points, depth, info, transform, transform_point, body_depth):
    result = {}
    if depth is None or transform is None:
        return result
    for joint, (u, v, score) in points.items():
        x, y = round(u), round(v)
        patch = depth[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
        valid = patch[np.isfinite(patch) & (patch > 0.25) & (patch < 7.5)]
        if len(valid) < 9 or np.percentile(valid, 80) - np.percentile(valid, 20) > 0.25:
            continue
        z = float(np.median(valid))
        if abs(z - body_depth) > 0.75:
            continue
        optical = np.array([(u - info.k[2]) * z / info.k[0], (v - info.k[5]) * z / info.k[4], z])
        world = transform_point(optical, transform)
        if np.all(np.isfinite(world)) and 0.02 < world[2] < 2.5:
            result[joint] = [*map(float, world), score]
    return result


def torso_orientation(joints, head_yaw=None):
    """Semantic left/right shoulder and hip baselines give a planar forward.

One baseline alone is ambiguous; require both to agree or a visible face to
confirm the sign. The quality is a gate, not a calibrated probability.
"""
    forwards, quality = [], []
    for right, left, minimum_height in ((2, 5, 0.85), (8, 11, 0.4)):
        if right not in joints or left not in joints:
            continue
        a, b = np.asarray(joints[right][:3]), np.asarray(joints[left][:3])
        axis = b - a
        width = float(np.linalg.norm(axis[:2]))
        if not 0.16 < width < 0.65 or abs(axis[2]) > 0.22 or min(a[2], b[2]) < minimum_height:
            continue
        forwards.append(np.array([axis[1], -axis[0]]) / width)
        quality.append(min(joints[right][3], joints[left][3]))
    if not forwards:
        return None, 0.0
    if len(forwards) == 2 and forwards[0] @ forwards[1] < math.cos(math.radians(35)):
        return None, 0.0
    direction = np.mean(forwards, axis=0)
    angle = math.atan2(direction[1], direction[0])
    if head_yaw is not None:
        alignment = math.cos(angle - head_yaw)
        if alignment < 0:
            angle = math.atan2(-direction[1], -direction[0])
        if abs(math.atan2(math.sin(angle - head_yaw), math.cos(angle - head_yaw))) > math.radians(70):
            return None, 0.0
    elif len(forwards) < 2:
        return None, 0.0
    # Both torso baselines, or one plus head evidence, passed the geometry gate.
    return angle, float(min(0.85, 0.45 + min(quality)))


def visibility(joints):
    upper = sum(j in joints for j in (1, 2, 3, 4, 5, 6, 7))
    lower = sum(j in joints for j in (8, 9, 10, 11, 12, 13))
    if upper >= 3:
        return 'full' if lower >= 3 else 'upper_body'
    return 'legs_only' if lower >= 2 else 'insufficient'
