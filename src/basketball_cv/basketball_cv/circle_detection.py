"""Find basketball candidates with Hough circles and simple visual checks."""

from dataclasses import dataclass

import cv2

import numpy as np


@dataclass(frozen=True)
class CircleConfig:
    """Parameters shared by Hough detection and candidate verification."""

    dp: float = 1.2
    min_dist: int = 70
    param1: int = 120
    param2: int = 30
    min_radius: int = 18
    max_radius: int = 0
    blur_kernel: int = 7
    min_edge_fraction: float = 0.18
    min_visible_fraction: float = 0.55
    min_score: float = 0.40


@dataclass(frozen=True)
class CircleResult:
    """The highest scoring accepted circle in a frame."""

    x: float
    y: float
    radius: float
    score: float


def find_basketball(bgr, config):
    """Return the best Hough circle and an edge diagnostic image."""
    if bgr is None or bgr.size == 0:
        raise ValueError('image must not be empty')
    if config.dp <= 0 or config.min_dist <= 0:
        raise ValueError('dp and min_dist must be positive')
    if config.min_radius < 0 or config.max_radius < 0:
        raise ValueError('radii must not be negative')
    if config.max_radius and config.max_radius < config.min_radius:
        raise ValueError('max_radius must be 0 or at least min_radius')

    # 1. 模糊灰度图，压低球面纹理、缝线及传感器噪声对霍夫投票的影响。
    blur_size = max(3, int(config.blur_kernel) | 1)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)

    # 2. 霍夫变换根据边缘投票找圆；这里不要求橙色区域连成一片。
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=config.dp,
        minDist=config.min_dist, param1=config.param1,
        param2=config.param2, minRadius=config.min_radius,
        maxRadius=config.max_radius,
    )
    # 3. 只使用几何边缘验证，不读取 HSV，也不计算颜色分数。
    edges = cv2.Canny(gray, config.param1 // 2, config.param1)
    edges = cv2.dilate(edges, np.ones((5, 5), np.uint8))
    if circles is None:
        return None, edges

    height, width = gray.shape
    best = None
    for x, y, radius in circles[0]:
        x, y, radius = float(x), float(y), float(radius)
        samples = max(120, int(2 * np.pi * radius / 2))
        angles = np.linspace(0, 2 * np.pi, samples, endpoint=False)
        px = np.rint(x + radius * np.cos(angles)).astype(int)
        py = np.rint(y + radius * np.sin(angles)).astype(int)
        visible = (px >= 0) & (px < width) & (py >= 0) & (py < height)
        if np.mean(visible) < config.min_visible_fraction:
            continue
        edge_fraction = np.mean(edges[py[visible], px[visible]] > 0)
        if edge_fraction < config.min_edge_fraction:
            continue

        # 纯霍夫测试版：分数就是圆周边缘支持率，不含颜色权重。
        score = edge_fraction
        if score >= config.min_score and (best is None or score > best.score):
            best = CircleResult(x, y, radius, float(score))

    return best, edges
