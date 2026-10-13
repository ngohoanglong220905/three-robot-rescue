"""Approximate head forward direction from five measured YuNet landmarks.

Generic face dimensions in metres, right/down axes, nose toward negative Z.
This estimates head orientation, not pupil gaze. No world/scenario input.
"""
import cv2
import numpy as np


FACE_POINTS = np.array([
    [-0.032, -0.030, 0.0], [0.032, -0.030, 0.0],
    [0.0, 0.0, -0.027],
    [-0.025, 0.030, -0.002], [0.025, 0.030, -0.002],
], dtype=np.float64)


def estimate_heads(detector, bgr, people, info, minimum_face_size=24):
    """Map person/negative standalone-face index to a measured head pose.

    Reject tiny, occluded/back-facing, implausible and poorly fitted faces.
    The PnP range is used only as a consistency check against measured depth.
    """
    if info is None or info.k[0] <= 0 or info.k[4] <= 0:
        return {}
    if (info.width, info.height) != (bgr.shape[1], bgr.shape[0]):
        return {}
    k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(info.d, dtype=np.float64) if len(info.d) else None
    detector.setInputSize((bgr.shape[1], bgr.shape[0]))
    _, faces = detector.detect(bgr)
    result = {}
    if faces is None:
        return result
    for face_id, face in enumerate(sorted(faces, key=lambda f: float(f[-1]), reverse=True)):
        x, y, w, h = map(float, face[:4])
        if min(w, h) < minimum_face_size or x < 1 or y < 1:
            continue
        if x + w >= bgr.shape[1] - 1 or y + h >= bgr.shape[0] - 1:
            continue
        landmarks = face[4:14].reshape(5, 2).astype(np.float64)
        nose = landmarks[2]
        candidates = [i for i, (x1, y1, x2, y2, _) in enumerate(people)
                      if x1 <= nose[0] <= x2 and y1 <= nose[1] <= y1 + 0.55 * (y2 - y1)]
        # A visible face does not require a YOLO whole-person box. It still
        # passes landmarks/PnP here and measured depth/height checks downstream.
        index = (min(candidates, key=lambda i: abs((people[i][0] + people[i][2]) / 2 - nose[0]))
                 if candidates else -face_id - 1)
        if index in result:
            continue
        if np.linalg.norm(landmarks[0] - landmarks[1]) < 8:
            continue
        try:
            ok, rotation, translation = cv2.solvePnP(
                FACE_POINTS, landmarks, k, distortion, flags=cv2.SOLVEPNP_EPNP)
            if not ok:
                continue
            ok, rotation, translation = cv2.solvePnP(
                FACE_POINTS, landmarks, k, distortion, rotation, translation,
                useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok or not np.all(np.isfinite(translation)) or not 0.3 < translation[2, 0] < 8:
                continue
            projected, _ = cv2.projectPoints(FACE_POINTS, rotation, translation, k, distortion)
            error = np.linalg.norm(projected.reshape(-1, 2) - landmarks, axis=1).mean()
            if not np.isfinite(error) or error > min(3.0, 0.07 * w):
                continue
            matrix, _ = cv2.Rodrigues(rotation)
            forward = matrix @ np.array([0.0, 0.0, -1.0])
            towards_camera = -translation.ravel() / np.linalg.norm(translation)
            if not np.all(np.isfinite(forward)) or np.dot(forward, towards_camera) < 0.15:
                continue
            result[index] = (forward, (x, y, x + w, y + h, float(face[-1])),
                             float(translation[2, 0]))
        except cv2.error:
            continue
    return result
