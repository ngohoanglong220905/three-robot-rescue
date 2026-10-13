"""Bounded RGB proposals and depth localisation for multicolour attention targets.

This is a visual heuristic, not a TV classifier. Person boxes are excluded;
depth/height gates reject most floor patterns and small robot decorations.
No authored world positions, materials, model names or labels are inputs.
"""
import cv2
import numpy as np


def hue_groups(hues):
    """Count substantial hue bands, suppressing neighbours and the red wrap."""
    if len(hues) < 40:
        return 0
    counts = np.bincount(np.asarray(hues, dtype=np.int32) // 10, minlength=18)
    minimum = max(8, 0.06 * len(hues))
    groups = 0
    while counts.max() >= minimum:
        peak = int(counts.argmax())
        groups += 1
        counts[[(peak - 1) % 18, peak, (peak + 1) % 18]] = 0
    return groups


def colorful_regions(bgr, people, maximum=4):
    """Propose connected saturated regions with >= 3 substantial hue bands.

    Work at <= 320 px width and retain <= 1024 coloured samples per region.
    Clipped objects are allowed; only their measured visible part is located.
    """
    scale = min(1.0, 320.0 / bgr.shape[1])
    image = cv2.resize(bgr, (round(bgr.shape[1] * scale), round(bgr.shape[0] * scale)))
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    allowed = np.ones(hsv.shape[:2], np.uint8)
    for x1, y1, x2, y2, _ in people:
        padx, pady = 0.04 * (x2 - x1), 0.03 * (y2 - y1)
        a = (max(0, int((x1 - padx) * scale)), max(0, int((y1 - pady) * scale)))
        b = (min(image.shape[1] - 1, int((x2 + padx) * scale)),
             min(image.shape[0] - 1, int((y2 + pady) * scale)))
        cv2.rectangle(allowed, a, b, 0, -1)
    coloured = ((hsv[..., 1] >= 55) & (hsv[..., 2] >= 45) & (allowed > 0)).astype(np.uint8)
    joined = cv2.morphologyEx(coloured, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    joined &= allowed
    count, labels, stats, _ = cv2.connectedComponentsWithStats(joined, connectivity=8)
    regions = []
    order = sorted(range(1, count), key=lambda i: int(stats[i, cv2.CC_STAT_AREA]), reverse=True)[:24]
    for index in order:
        x, y, width, height, area = map(int, stats[index])
        if area < max(80, image.shape[0] * image.shape[1] * 0.002) or min(width, height) < 12:
            continue
        yy, xx = np.nonzero((labels == index) & (coloured > 0))
        if len(xx) < 0.25 * width * height:
            continue
        hues = hsv[yy, xx, 0]
        if hue_groups(hues) < 3:
            continue
        selection = np.linspace(0, len(xx) - 1, min(1024, len(xx)), dtype=int)
        uv = (np.column_stack((xx[selection], yy[selection])) + 0.5) / scale - 0.5
        regions.append(dict(box=(x / scale, y / scale, (x + width) / scale, (y + height) / scale),
                            uv=uv, hues=hues[selection]))
        if len(regions) >= maximum:
            break
    return regions


def locate_colorful_regions(regions, depth, info, transform, transform_point):
    """Require actual colour samples on a coherent elevated RGB-D surface."""
    located = []
    if depth is None or info is None or transform is None or info.k[0] <= 0 or info.k[4] <= 0:
        return located
    for region in regions:
        uv = region['uv']
        pixels = np.rint(uv).astype(int)
        pixels[:, 0] = np.clip(pixels[:, 0], 0, depth.shape[1] - 1)
        pixels[:, 1] = np.clip(pixels[:, 1], 0, depth.shape[0] - 1)
        ranges = depth[pixels[:, 1], pixels[:, 0]]
        valid = np.isfinite(ranges) & (ranges > 0.25) & (ranges < 7.5)
        if np.count_nonzero(valid) < max(40, len(ranges) * 0.45):
            continue
        uv, ranges, hues = uv[valid], ranges[valid], region['hues'][valid]
        optical = np.column_stack(((uv[:, 0] - info.k[2]) * ranges / info.k[0],
                                   (uv[:, 1] - info.k[5]) * ranges / info.k[4], ranges))
        if not np.isfinite(optical).all():
            continue
        # A close oblique screen has a large depth gradient across its width.
        # Accept a coherent tilted plane before falling back to a depth band;
        # a fixed narrow depth band alone would throw away its other colours.
        centered = optical - optical.mean(axis=0)
        _, _, axes = np.linalg.svd(centered, full_matrices=False)
        residual = np.abs(centered @ axes[-1])
        coherent = (residual < 0.10 if np.percentile(residual, 90) < 0.08 else
                    np.abs(ranges - np.median(ranges)) < 0.40)
        if np.count_nonzero(coherent) < max(40, len(ranges) * 0.45):
            continue
        optical, hues = optical[coherent], hues[coherent]
        world = transform_point(optical, transform)
        valid_height = np.isfinite(world).all(axis=1) & (world[:, 2] > 0.45) & (world[:, 2] < 3.0)
        if np.count_nonzero(valid_height) < 40:
            continue
        world, hues = world[valid_height], hues[valid_height]
        colours = hue_groups(hues)
        if colours < 3:
            continue
        low, high = np.percentile(world, [5, 95], axis=0)
        horizontal = float(np.linalg.norm(high[:2] - low[:2]))
        if not (0.20 < horizontal < 4.0 and 0.18 < high[2] - low[2] < 2.4 and high[2] > 0.75):
            continue
        # Trim individual noisy depth pixels before taking the XY silhouette.
        trimmed = world[np.all((world >= low) & (world <= high), axis=1)]
        if len(trimmed) < 16:
            continue
        boundary = cv2.convexHull(trimmed[:, :2].astype(np.float32)).reshape(-1, 2)
        if len(boundary) > 16:
            boundary = boundary[np.linspace(0, len(boundary) - 1, 16, dtype=int)]
        located.append(dict(position=np.median(trimmed, axis=0).astype(float).tolist(),
                            footprint=boundary.astype(float).tolist(), hue_groups=colours,
                            box=list(map(float, region['box'])), source='rgbd_multicolour_surface'))
    return located
