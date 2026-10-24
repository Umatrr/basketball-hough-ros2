"""Check representative detection cases without ROS or a physical camera."""

from dataclasses import replace

from basketball_cv.circle_detection import CircleConfig, find_basketball

import cv2

import numpy as np


def make_ball(center, radius):
    """Draw an orange ball with dark seams against a plain background."""
    image = np.full((480, 640, 3), 60, dtype=np.uint8)
    # HSV 约为 H=11, S=193, V=145，接近棕红篮球而非浅橙色。
    cv2.circle(image, center, radius, (35, 75, 145), -1)
    cv2.circle(image, center, radius, (15, 30, 40), 4)
    for angle in (0, 45, 90, 135):
        theta = np.deg2rad(angle)
        offset = np.array([np.cos(theta), np.sin(theta)]) * radius * 0.88
        start = tuple(np.rint(np.array(center) - offset).astype(int))
        end = tuple(np.rint(np.array(center) + offset).astype(int))
        cv2.line(image, start, end, (15, 30, 40), max(4, radius // 12))
    return image


def test_dark_seams_and_cropped_close_ball():
    """Deep seams and a partially clipped circle should stay detectable."""
    for center, radius in [((320, 240), 105), ((80, 240), 165)]:
        image = make_ball(center, radius)
        circle, mask = find_basketball(image, CircleConfig())
        assert circle is not None
        assert abs(circle.x - center[0]) < 10
        assert abs(circle.y - center[1]) < 10
        assert abs(circle.radius - radius) < 15
        assert mask.shape == (480, 640)
        assert 0.0 <= circle.edge_score <= 1.0
        assert 0.0 <= circle.color_score <= 1.0


def test_no_circle():
    """A uniform frame should not create a detection."""
    blank = np.full((480, 640, 3), 60, dtype=np.uint8)
    circle, mask = find_basketball(blank, CircleConfig())
    assert circle is None
    assert not np.any(mask)


def test_debug_keeps_candidate_that_fails_color_gate():
    """Debug data exposes a geometric candidate rejected by color threshold."""
    config = replace(CircleConfig(), min_color_fraction=1.01)
    circle, _ = find_basketball(make_ball((320, 240), 105), config)
    assert circle is not None
    assert not circle.accepted
    assert circle.edge_score > 0
    assert circle.color_score > 0


def test_ball_wins_over_similarly_colored_face():
    """A real circular outline should outrank an orange oval face."""
    image = make_ball((440, 240), 90)
    cv2.ellipse(image, (140, 230), (70, 95), 0, 0, 360,
                (80, 145, 200), -1)
    cv2.ellipse(image, (140, 230), (70, 95), 0, 0, 360,
                (30, 45, 50), 3)
    cv2.circle(image, (115, 210), 9, (20, 20, 20), -1)
    cv2.circle(image, (165, 210), 9, (20, 20, 20), -1)

    circle, _ = find_basketball(image, CircleConfig())
    assert circle is not None
    assert abs(circle.x - 440) < 10
