"""
Catmull-Rom spline path smoothing (small, dependency-free).
"""

import math
import numpy as np


def catmull_rom_spline(points, samples_per_segment=20, closed=False):
    """
    points: list of (x, y)
    returns: list of (x, y) smoothed
    """
    pts = [tuple(p) for p in points]
    if len(pts) < 2:
        return pts

    # extend endpoints for spline calculation
    if closed:
        pts_ext = pts[-1:] + pts + pts[:2]
    else:
        pts_ext = [pts[0]] + pts + [pts[-1]]

    out = []
    for i in range(len(pts_ext) - 3):
        P0 = np.array(pts_ext[i])
        P1 = np.array(pts_ext[i + 1])
        P2 = np.array(pts_ext[i + 2])
        P3 = np.array(pts_ext[i + 3])
        for j in range(samples_per_segment):
            t = j / float(samples_per_segment)
            t2 = t * t
            t3 = t2 * t
            # Catmull-Rom formula
            point = 0.5 * (
                (2 * P1)
                + (-P0 + P2) * t
                + (2 * P0 - 5 * P1 + 4 * P2 - P3) * t2
                + (-P0 + 3 * P1 - 3 * P2 + P3) * t3
            )
            out.append((float(point[0]), float(point[1])))
    # ensure last point included
    out.append((pts[-1][0], pts[-1][1]))
    return out
