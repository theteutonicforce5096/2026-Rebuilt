#!/usr/bin/env python3

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import dataclasses
import itertools
import json
import math
import os
import statistics
import subprocess
import sys
import tempfile
import time
from collections.abc import Iterable, Sequence
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from constants.shot_calculator_constants import default_r, default_y_dis, default_θ


CSV_DEFAULT = Path.home() / "Downloads" / "Shooter Table - Sheet1.csv"
VIDEO_DEFAULTS = tuple(
    Path.home() / "Downloads" / name
    for name in (
        "IMG_3318.mov",
        "IMG_3320.mov",
        "IMG_3321.mov",
        "IMG_3323.mov",
        "IMG_3324.mov",
        "IMG_3326.mov",
        "IMG_3327.mov",
        "IMG_3331.mov",
        "IMG_3333.mov",
        "IMG_3334.mov",
    )
)

CAPTURE_FPS = 240.0
VIDEO_FRAME_SCALE = 0.5
VIDEO_FRAME_STRIDE = 2
HUB_FRONT_TO_CENTER_IN = 23.51
HUB_FRONT_TO_CENTER_M = HUB_FRONT_TO_CENTER_IN * 0.0254
TARGET_HEIGHT_M = 72.0 * 0.0254
EXIT_HEIGHT_M = TARGET_HEIGHT_M - default_y_dis
WHEEL_DIAMETER_M = default_r * 2.0
TARGET_RPS_MIN = 0.0
TARGET_RPS_MAX = 113.0
TARGET_RPM_MAX = TARGET_RPS_MAX * 60.0
TARGET_RPM_MIN = TARGET_RPS_MIN * 60.0

BALL_MASS_KG = 0.215
BALL_DIAMETER_M = 0.1501
AIR_DENSITY = 1.225
DEFAULT_DT = 0.001
DEFAULT_MAX_SIM_TIME = 12.0
BINARY_SEARCH_ITERS = 40
HEIGHT_TOLERANCE_M = 0.02
REACHABLE_HEIGHT_TOLERANCE_M = 0.10
DEFAULT_MAX_RUNTIME_MINUTES = 30.0

LUT_START_M = 1.0
LUT_END_M = 10.0
LUT_STEP_M = 0.05
OBJECTIVE_SAMPLE_STEP_M = 0.25
OBJECTIVE_SAMPLE_DISTANCES = np.round(np.arange(LUT_START_M, LUT_END_M + 1e-9, OBJECTIVE_SAMPLE_STEP_M), 2)
FULL_LUT_DISTANCES = np.round(np.arange(LUT_START_M, LUT_END_M + 1e-9, LUT_STEP_M), 2)
ROBUSTNESS_MONTE_CARLO_SAMPLES = 256

PROXY_COARSE_STEP = 0.04
PROXY_GLOBAL_SAMPLES_PER_ANGLE = 3000
REFINE_STEP = 0.01
FINAL_STEP = 0.002

SLIP_MIN = 0.30
SLIP_MAX = 0.70
DRAG_MIN = 0.05
DRAG_MAX = 0.80
MAGNUS_MIN = -0.40
MAGNUS_MAX = 0.40

ANGLE_MIN_DEG = 58.0
ANGLE_MAX_DEG = 70.0
ANGLE_PROXY_STEP_DEG = 1.0
ANGLE_REFINE_STEP_DEG = 0.5
GLOBAL_SEED_COUNT = 8
LOCAL_REFINED_COUNT = 2

BASELINE_SLIP_FACTOR = 0.448
BASELINE_DRAG_COEFF = 0.104
BASELINE_MAGNUS_COEFF = -0.4
BASELINE_LAUNCH_ANGLE_DEG = 67.5
MAX_SOTM_SPEED_MPS = 3.0
HUB_ENTRY_RADIUS_M = 0.5295
BALL_RADIUS_M = BALL_DIAMETER_M / 2.0
USABLE_SCORING_RADIUS_M = HUB_ENTRY_RADIUS_M - BALL_RADIUS_M

YELLOW_LOW = np.array((15, 80, 80), dtype=np.uint8)
YELLOW_HIGH = np.array((45, 255, 255), dtype=np.uint8)


@dataclasses.dataclass(frozen=True, slots=True)
class CalibrationPoint:
    source_distance_in: float
    center_distance_m: float
    flywheel_rps: float


@dataclasses.dataclass(frozen=True, slots=True)
class VideoSummary:
    video_name: str
    shot_tofs_sec: tuple[float, ...]
    median_tof_sec: float
    ignored_shots: int


@dataclasses.dataclass(frozen=True, slots=True)
class TrackSummary:
    start_frame: int
    end_frame: int
    frames: tuple[int, ...]
    points: tuple[tuple[float, float], ...]
    start_x: float
    end_x: float
    start_y: float
    end_y: float
    min_y: float
    observed_tof_sec: float
    estimated_tof_sec: float
    horizontal_span_px: float
    vertical_lift_px: float
    score: float


@dataclasses.dataclass(frozen=True, slots=True)
class FitCandidate:
    slip_factor: float
    drag_coeff: float
    magnus_coeff: float
    launch_angle_deg: float
    loss: float
    rps_rms_relative: float
    tof_rms_relative: float
    matched_video_indices: tuple[int, ...]
    unmatched_distance_indices: tuple[int, ...]


@dataclasses.dataclass(frozen=True, slots=True)
class JavaVerification:
    max_rpm_abs_error: float
    max_tof_abs_error: float
    cases: tuple[tuple[float, float, float, float, float], ...]


@dataclasses.dataclass(frozen=True, slots=True)
class CandidateAssessment:
    candidate: FitCandidate
    rps_rms_relative: float
    tof_rms_relative: float
    max_adj_rps: float
    max_adj_tof: float
    rps_curvature_rms: float
    tof_curvature_rms: float
    empirical_score_mean: float
    empirical_score_min: float
    interpolated_score_mean: float
    interpolated_score_min: float
    overall_score_min: float
    rps_relative_p90: float
    tof_relative_p90: float
    matched_video_indices: tuple[int, ...]
    unmatched_distance_indices: tuple[int, ...]
    accepted: bool
    reasons: tuple[str, ...]
    lut_entries: tuple[tuple[float, float, float], ...]
    empirical_probabilities: tuple[tuple[float, float], ...]
    interpolated_probabilities: tuple[tuple[float, float], ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate the hybrid shot LUT from CSV + slow-mo video.")
    parser.add_argument("--csv", type=Path, default=CSV_DEFAULT)
    parser.add_argument("--video", dest="videos", action="append", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--skip-java-check", action="store_true")
    parser.add_argument("--max-runtime-minutes", type=float, default=DEFAULT_MAX_RUNTIME_MINUTES)
    return parser.parse_args()


def _baseline_candidate() -> FitCandidate:
    return FitCandidate(
        slip_factor=BASELINE_SLIP_FACTOR,
        drag_coeff=BASELINE_DRAG_COEFF,
        magnus_coeff=BASELINE_MAGNUS_COEFF,
        launch_angle_deg=BASELINE_LAUNCH_ANGLE_DEG,
        loss=0.0,
        rps_rms_relative=0.0,
        tof_rms_relative=0.0,
        matched_video_indices=(),
        unmatched_distance_indices=(),
    )


def load_empirical_points(csv_path: Path) -> tuple[CalibrationPoint, ...]:
    points: list[CalibrationPoint] = []
    with csv_path.open(newline="") as csv_file:
        reader = csv.reader(csv_file)
        next(reader)
        for row in reader:
            if len(row) < 2:
                continue
            raw_distance_in = float(row[0].split()[0])
            flywheel_rps = float(row[1])
            center_distance_m = (raw_distance_in + HUB_FRONT_TO_CENTER_IN) * 0.0254
            points.append(
                CalibrationPoint(
                    source_distance_in=raw_distance_in,
                    center_distance_m=center_distance_m,
                    flywheel_rps=flywheel_rps,
                )
            )

    return tuple(sorted(points, key=lambda point: point.center_distance_m))


def _component_candidates(mask: np.ndarray) -> list[tuple[float, float, int, int, int, int]]:
    component_count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    candidates: list[tuple[float, float, int, int, int, int]] = []
    for component in range(1, component_count):
        area = int(stats[component, cv2.CC_STAT_AREA])
        if area < 18 or area > 2500:
            continue
        left = int(stats[component, cv2.CC_STAT_LEFT])
        top = int(stats[component, cv2.CC_STAT_TOP])
        width = int(stats[component, cv2.CC_STAT_WIDTH])
        height = int(stats[component, cv2.CC_STAT_HEIGHT])
        center_x, center_y = centroids[component]
        candidates.append((float(center_x), float(center_y), area, left, top, width))
    return candidates


def _fit_reference_planes(track_summaries: Sequence[TrackSummary], width: int) -> tuple[float, float]:
    anchor_tracks = [
        track
        for track in track_summaries
        if track.horizontal_span_px > width * 0.08 and track.vertical_lift_px > 20.0
    ]
    if not anchor_tracks:
        return width * 0.88, width * 0.16

    spans = [track.horizontal_span_px for track in anchor_tracks]
    median_span = float(np.percentile(spans, 60))
    shooter_observed_x = float(np.percentile([track.start_x for track in anchor_tracks], 90))
    hub_observed_x = float(np.percentile([track.end_x for track in anchor_tracks], 10))

    shooter_margin = max(width * 0.015, median_span * 0.08)
    hub_margin = max(width * 0.020, median_span * 0.10)

    shooter_reference_x = float(
        np.clip(
            max(shooter_observed_x + shooter_margin, width * 0.82),
            width * 0.74,
            width * 0.95,
        )
    )
    hub_reference_x = float(
        np.clip(
            min(hub_observed_x - hub_margin, width * 0.18),
            width * 0.06,
            width * 0.30,
        )
    )
    if hub_reference_x >= shooter_reference_x - width * 0.18:
        hub_reference_x = shooter_reference_x - width * 0.18

    return shooter_reference_x, hub_reference_x


def _estimate_track_tof(
    frames: Sequence[int],
    points: Sequence[tuple[float, float]],
    shooter_reference_x: float,
    hub_reference_x: float,
) -> float:
    observed_tof_sec = (frames[-1] - frames[0]) / CAPTURE_FPS
    if len(frames) < 6:
        return observed_tof_sec

    times = np.array([(frame - frames[0]) / CAPTURE_FPS for frame in frames], dtype=np.float64)
    xs = np.array([point[0] for point in points], dtype=np.float64)
    segment_size = max(3, len(times) // 4)
    early_time = float(np.mean(times[:segment_size]))
    late_time = float(np.mean(times[-segment_size:]))
    if late_time - early_time < 1e-6:
        return observed_tof_sec

    endpoint_velocity_x = (
        float(np.mean(xs[-segment_size:])) - float(np.mean(xs[:segment_size]))
    ) / (late_time - early_time)
    endpoint_intercept_x = float(np.mean(xs[:segment_size])) - endpoint_velocity_x * early_time

    linear_velocity_x, linear_intercept_x = np.polyfit(times, xs, 1)
    if linear_velocity_x < -15.0 and endpoint_velocity_x < -15.0:
        velocity_x = 0.5 * (linear_velocity_x + endpoint_velocity_x)
        intercept_x = 0.5 * (linear_intercept_x + endpoint_intercept_x)
    elif linear_velocity_x < -15.0:
        velocity_x = linear_velocity_x
        intercept_x = linear_intercept_x
    else:
        velocity_x = endpoint_velocity_x
        intercept_x = endpoint_intercept_x

    if velocity_x >= -15.0:
        return observed_tof_sec

    observed_end_time = float(times[-1])
    estimated_launch_time = min((shooter_reference_x - intercept_x) / velocity_x, 0.0)
    estimated_score_time = max((hub_reference_x - intercept_x) / velocity_x, observed_end_time)
    estimated_tof_sec = estimated_score_time - estimated_launch_time

    if estimated_score_time < observed_end_time - 0.02:
        return observed_tof_sec
    if estimated_tof_sec < observed_tof_sec or estimated_tof_sec > observed_tof_sec * 4.0:
        return observed_tof_sec
    if not math.isfinite(estimated_tof_sec):
        return observed_tof_sec

    expected_span_px = max(shooter_reference_x - hub_reference_x, 1.0)
    observed_span_px = max(xs[0] - xs[-1], 0.0)
    observed_coverage = float(np.clip(observed_span_px / expected_span_px, 0.0, 1.0))
    blend = 0.25 + 0.30 * observed_coverage
    blended_tof_sec = observed_tof_sec + blend * (estimated_tof_sec - observed_tof_sec)
    return float(min(blended_tof_sec, observed_tof_sec * 2.5))


def _weighted_percentile(values: Sequence[float], weights: Sequence[float], quantile: float) -> float:
    if not values:
        raise ValueError("Cannot compute a percentile from an empty sequence")

    ordered = sorted(zip(values, weights, strict=True), key=lambda pair: pair[0])
    total_weight = sum(max(weight, 0.0) for _, weight in ordered)
    if total_weight <= 0.0:
        return float(statistics.median(values))

    threshold = total_weight * quantile
    running_weight = 0.0
    for value, weight in ordered:
        running_weight += max(weight, 0.0)
        if running_weight >= threshold:
            return float(value)

    return float(ordered[-1][0])


def _track_score(
    track: TrackSummary,
    shooter_reference_x: float,
    hub_reference_x: float,
    width: int,
    height: int,
) -> float:
    start_alignment = 1.0 - min(1.0, abs(track.start_x - shooter_reference_x) / (width * 0.30))
    end_alignment = 1.0 - min(1.0, abs(track.end_x - hub_reference_x) / (width * 0.30))
    duration_score = min(track.estimated_tof_sec / 1.0, 1.5)
    span_score = min(track.horizontal_span_px / (width * 0.35), 1.5)
    lift_score = min(track.vertical_lift_px / (height * 0.35), 1.5)
    visibility_bonus = 0.4 if track.end_y < height * 0.72 else 0.0

    return (
        2.0 * duration_score
        + 2.0 * span_score
        + 1.5 * lift_score
        + 1.0 * start_alignment
        + 1.0 * end_alignment
        + visibility_bonus
    )


def _dedupe_tracks(track_summaries: Sequence[TrackSummary]) -> list[TrackSummary]:
    selected: list[TrackSummary] = []
    for track in sorted(track_summaries, key=lambda candidate: candidate.score, reverse=True):
        overlaps_existing = False
        for existing in selected:
            overlap_start = max(track.start_frame, existing.start_frame)
            overlap_end = min(track.end_frame, existing.end_frame)
            if overlap_end < overlap_start:
                continue
            overlap = overlap_end - overlap_start + 1
            shorter = min(
                track.end_frame - track.start_frame + 1,
                existing.end_frame - existing.start_frame + 1,
            )
            if shorter > 0 and overlap / shorter > 0.45:
                overlaps_existing = True
                break
        if not overlaps_existing:
            selected.append(track)

    return sorted(selected, key=lambda track: track.start_frame)


def _extract_track_summaries(video_path: Path, *, relaxed: bool = False) -> tuple[TrackSummary, ...]:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise ValueError(f"Unable to open {video_path}")

    ok, frame = capture.read()
    if not ok:
        raise ValueError(f"Unable to read first frame from {video_path}")

    frame = cv2.resize(
        frame,
        None,
        fx=VIDEO_FRAME_SCALE,
        fy=VIDEO_FRAME_SCALE,
        interpolation=cv2.INTER_AREA,
    )
    height, width = frame.shape[:2]
    prev_gray: np.ndarray | None = None
    active_tracks: list[dict[str, object]] = []
    finished_tracks: list[dict[str, object]] = []

    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    capture.set(cv2.CAP_PROP_POS_FRAMES, 0)

    for frame_index in range(frame_count):
        ok, frame = capture.read()
        if not ok:
            break

        if frame_index % VIDEO_FRAME_STRIDE != 0:
            continue

        frame = cv2.resize(
            frame,
            None,
            fx=VIDEO_FRAME_SCALE,
            fy=VIDEO_FRAME_SCALE,
            interpolation=cv2.INTER_AREA,
        )
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        yellow_mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
        yellow_mask = cv2.medianBlur(yellow_mask, 5)

        if prev_gray is None:
            motion_mask = yellow_mask
        else:
            delta = cv2.absdiff(gray, prev_gray)
            _, motion_mask = cv2.threshold(delta, 12, 255, cv2.THRESH_BINARY)
            motion_mask = cv2.dilate(motion_mask, None, iterations=2)
            motion_mask = cv2.erode(motion_mask, None, iterations=1)
        prev_gray = gray

        moving_yellow = cv2.bitwise_and(yellow_mask, motion_mask)
        moving_yellow = cv2.morphologyEx(moving_yellow, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        candidates = []
        for center_x, center_y, area, left, top, component_width in _component_candidates(moving_yellow):
            if center_y > height * 0.88:
                continue
            if center_x > width * 0.58 and center_y > height * 0.56:
                continue
            if center_x < width * 0.05 or left + component_width >= width - 4 or top <= 4:
                continue
            candidates.append((center_x, center_y, area))

        used = [False] * len(candidates)
        next_tracks: list[dict[str, object]] = []
        for track in active_tracks:
            last_x, last_y = track["points"][-1]
            best_index = None
            best_distance = float("inf")
            for candidate_index, (center_x, center_y, _) in enumerate(candidates):
                if used[candidate_index]:
                    continue
                distance = math.hypot(center_x - last_x, center_y - last_y)
                if distance < best_distance:
                    best_distance = distance
                    best_index = candidate_index

            if best_index is not None and best_distance < 70.0:
                used[best_index] = True
                center_x, center_y, _ = candidates[best_index]
                track["frames"].append(frame_index)
                track["points"].append((center_x, center_y))
                track["missed"] = 0
                next_tracks.append(track)
                continue

            track["missed"] += 1
            if track["missed"] <= 3:
                next_tracks.append(track)
            else:
                finished_tracks.append(track)

        active_tracks = next_tracks

        for candidate_index, (center_x, center_y, _) in enumerate(candidates):
            if used[candidate_index]:
                continue
            active_tracks.append(
                {
                    "frames": [frame_index],
                    "points": [(center_x, center_y)],
                    "missed": 0,
                }
            )

    capture.release()
    finished_tracks.extend(active_tracks)

    raw_tracks: list[TrackSummary] = []
    for track in finished_tracks:
        frames = track["frames"]
        points = track["points"]
        if len(frames) < 6:
            continue

        start_x, start_y = points[0]
        end_x, end_y = points[-1]
        min_y = min(y for _, y in points)
        observed_tof_sec = (frames[-1] - frames[0]) / CAPTURE_FPS
        if not 0.08 <= observed_tof_sec <= 3.5:
            continue
        horizontal_span_px = start_x - end_x
        vertical_lift_px = start_y - min_y
        if horizontal_span_px < 20.0:
            continue
        if start_x < width * 0.20:
            continue

        raw_tracks.append(
            TrackSummary(
                start_frame=frames[0],
                end_frame=frames[-1],
                frames=tuple(frames),
                points=tuple((float(x), float(y)) for x, y in points),
                start_x=float(start_x),
                end_x=float(end_x),
                start_y=float(start_y),
                end_y=float(end_y),
                min_y=float(min_y),
                observed_tof_sec=float(observed_tof_sec),
                estimated_tof_sec=float(observed_tof_sec),
                horizontal_span_px=float(horizontal_span_px),
                vertical_lift_px=float(vertical_lift_px),
                score=0.0,
            )
        )

    shooter_reference_x, hub_reference_x = _fit_reference_planes(raw_tracks, width)

    scored_tracks: list[TrackSummary] = []
    for track in raw_tracks:
        estimated_tof_sec = _estimate_track_tof(
            frames=track.frames,
            points=track.points,
            shooter_reference_x=shooter_reference_x,
            hub_reference_x=hub_reference_x,
        )
        track = dataclasses.replace(track, estimated_tof_sec=estimated_tof_sec)
        track = dataclasses.replace(
            track,
            score=_track_score(track, shooter_reference_x, hub_reference_x, width, height),
        )

        if relaxed:
            if track.start_x <= width * 0.58:
                continue
            if track.horizontal_span_px < width * 0.08:
                continue
            if track.vertical_lift_px < 20.0 and track.end_y > height * 0.76:
                continue
            if track.score < 3.4:
                continue
        else:
            if track.start_x <= width * 0.28:
                continue
            if track.horizontal_span_px < width * 0.06:
                continue
            if track.vertical_lift_px < 8.0 and track.end_y > height * 0.70:
                continue
            if track.score < 2.8:
                continue

        scored_tracks.append(track)

    return tuple(_dedupe_tracks(scored_tracks))


def _detect_shot_tofs(video_path: Path, *, relaxed: bool = False) -> tuple[float, ...]:
    return tuple(track.estimated_tof_sec for track in _extract_track_summaries(video_path, relaxed=relaxed))


def summarize_video(video_path: Path) -> VideoSummary:
    tracks = _extract_track_summaries(video_path)
    if len(tracks) < 2:
        tracks = _extract_track_summaries(video_path, relaxed=True)
    shot_tofs = tuple(track.estimated_tof_sec for track in tracks)
    if len(shot_tofs) < 2:
        raise ValueError(f"Unable to extract enough scored shots from {video_path.name}")

    usable_tracks = tracks[1:]
    usable_tofs = tuple(track.estimated_tof_sec for track in usable_tracks)
    representative_tof = max(
        float(statistics.median(usable_tofs)),
        _weighted_percentile(
            usable_tofs,
            [track.score * max(track.horizontal_span_px, 1.0) for track in usable_tracks],
            0.60,
        ),
    )
    return VideoSummary(
        video_name=video_path.name,
        shot_tofs_sec=usable_tofs,
        median_tof_sec=representative_tof,
        ignored_shots=1,
    )


def load_video_summaries(video_paths: Sequence[Path]) -> tuple[VideoSummary, ...]:
    max_workers = min(len(video_paths), os.cpu_count() or 1, 4)
    if max_workers <= 1:
        summaries = [summarize_video(path) for path in video_paths]
    else:
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            summaries = list(executor.map(summarize_video, video_paths))
    return tuple(sorted(summaries, key=lambda summary: summary.median_tof_sec))


def _drag_compensated_time(tof: np.ndarray | float, drag_coeff: float = 0.47) -> np.ndarray:
    tof_array = np.asarray(tof, dtype=np.float64)
    tof_array = np.maximum(tof_array, 0.0)
    if drag_coeff < 1e-6:
        return tof_array
    return (1.0 - np.exp(-drag_coeff * tof_array)) / drag_coeff


def _smoothness_stats(
    distances: Sequence[float],
    rps_values: Sequence[float],
    tof_values: Sequence[float],
) -> tuple[float, float, float, float, float]:
    if len(distances) < 3:
        return 0.0, 0.0, 0.0, 0.0, 0.0

    distance_array = np.asarray(distances, dtype=np.float64)
    rps_array = np.asarray(rps_values, dtype=np.float64)
    tof_array = np.asarray(tof_values, dtype=np.float64)

    step_m = float(np.median(np.diff(distance_array)))
    step_scale = step_m / LUT_STEP_M
    rps_jump_threshold = 1.5 * step_scale
    tof_jump_threshold = 0.03 * step_scale

    adj_rps = np.diff(rps_array)
    adj_tof = np.diff(tof_array)
    curvature_rps = np.diff(rps_array, n=2)
    curvature_tof = np.diff(tof_array, n=2)

    max_adj_rps = float(np.max(np.abs(adj_rps)))
    max_adj_tof = float(np.max(adj_tof))
    rps_curvature_rms = float(math.sqrt(float(np.mean(np.square(curvature_rps))))) if curvature_rps.size else 0.0
    tof_curvature_rms = float(math.sqrt(float(np.mean(np.square(curvature_tof))))) if curvature_tof.size else 0.0

    rps_jump_penalty = float(
        np.mean(np.square(np.clip(np.abs(adj_rps) / max(rps_jump_threshold, 1e-9) - 1.0, 0.0, None)))
    )
    tof_jump_penalty = float(
        np.mean(np.square(np.clip(adj_tof / max(tof_jump_threshold, 1e-9) - 1.0, 0.0, None)))
    )
    curvature_penalty = (
        0.01 * rps_curvature_rms / max(rps_jump_threshold, 1e-9)
        + 0.25 * tof_curvature_rms / max(tof_jump_threshold, 1e-9)
    )
    return (
        max_adj_rps,
        max_adj_tof,
        rps_curvature_rms,
        tof_curvature_rms,
        rps_jump_penalty + tof_jump_penalty + curvature_penalty,
    )


def _parameter_edge_penalty(candidate: FitCandidate) -> float:
    penalties = []
    for value, low, high in (
        (candidate.slip_factor, SLIP_MIN, SLIP_MAX),
        (candidate.drag_coeff, DRAG_MIN, DRAG_MAX),
        (candidate.magnus_coeff, MAGNUS_MIN, MAGNUS_MAX),
        (candidate.launch_angle_deg, ANGLE_MIN_DEG, ANGLE_MAX_DEG),
    ):
        span = max(high - low, 1e-9)
        edge_fraction = min((value - low) / span, (high - value) / span)
        penalties.append(max(0.0, 0.08 - edge_fraction) / 0.08)

    return 0.02 * float(sum(penalty * penalty for penalty in penalties))


def _area_for_ball(ball_diameter_m: float) -> float:
    radius = ball_diameter_m / 2.0
    return math.pi * radius * radius


def _k_drag(drag_coeff: np.ndarray) -> np.ndarray:
    area = _area_for_ball(BALL_DIAMETER_M)
    return (AIR_DENSITY * drag_coeff * area) / (2.0 * BALL_MASS_KG)


def _k_magnus(magnus_coeff: np.ndarray) -> np.ndarray:
    area = _area_for_ball(BALL_DIAMETER_M)
    return (AIR_DENSITY * magnus_coeff * area) / (2.0 * BALL_MASS_KG)


def _simulate_batch(
    exit_velocity_mps: np.ndarray,
    target_distance_m: np.ndarray,
    k_drag: np.ndarray,
    k_magnus: np.ndarray,
    launch_angle_deg: float,
    *,
    dt: float = DEFAULT_DT,
    max_time: float = DEFAULT_MAX_SIM_TIME,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    max_speed_mps = 250.0
    theta = math.radians(launch_angle_deg)
    vx = exit_velocity_mps * math.cos(theta)
    vz = exit_velocity_mps * math.sin(theta)
    x = np.zeros_like(exit_velocity_mps, dtype=np.float64)
    z = np.full_like(exit_velocity_mps, EXIT_HEIGHT_M, dtype=np.float64)

    reached = np.zeros_like(exit_velocity_mps, dtype=bool)
    z_at_target = np.full_like(exit_velocity_mps, np.nan, dtype=np.float64)
    tof_at_target = np.full_like(exit_velocity_mps, np.nan, dtype=np.float64)
    alive = np.ones_like(exit_velocity_mps, dtype=bool)

    def derivatives(state_x: np.ndarray, state_z: np.ndarray, state_vx: np.ndarray, state_vz: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        safe_vx = np.nan_to_num(state_vx, nan=0.0, posinf=0.0, neginf=0.0)
        safe_vz = np.nan_to_num(state_vz, nan=0.0, posinf=0.0, neginf=0.0)
        speed = np.nan_to_num(np.hypot(safe_vx, safe_vz), nan=0.0, posinf=max_speed_mps, neginf=0.0)
        speed = np.minimum(speed, max_speed_mps)
        ax = -k_drag_alive * speed * safe_vx
        az = -9.81 - k_drag_alive * speed * safe_vz + k_magnus_alive * speed * speed
        return state_vx, state_vz, ax, az

    max_steps = int(max_time / dt)
    current_time = 0.0

    for _ in range(max_steps):
        if not np.any(alive):
            break

        alive_indices = np.flatnonzero(alive)
        k_drag_alive = k_drag[alive_indices]
        k_magnus_alive = k_magnus[alive_indices]

        state_x = x[alive_indices]
        state_z = z[alive_indices]
        state_vx = vx[alive_indices]
        state_vz = vz[alive_indices]

        k1x, k1z, k1vx, k1vz = derivatives(state_x, state_z, state_vx, state_vz)
        s2x = state_x + k1x * dt / 2.0
        s2z = state_z + k1z * dt / 2.0
        s2vx = state_vx + k1vx * dt / 2.0
        s2vz = state_vz + k1vz * dt / 2.0

        k2x, k2z, k2vx, k2vz = derivatives(s2x, s2z, s2vx, s2vz)
        s3x = state_x + k2x * dt / 2.0
        s3z = state_z + k2z * dt / 2.0
        s3vx = state_vx + k2vx * dt / 2.0
        s3vz = state_vz + k2vz * dt / 2.0

        k3x, k3z, k3vx, k3vz = derivatives(s3x, s3z, s3vx, s3vz)
        s4x = state_x + k3x * dt
        s4z = state_z + k3z * dt
        s4vx = state_vx + k3vx * dt
        s4vz = state_vz + k3vz * dt

        k4x, k4z, k4vx, k4vz = derivatives(s4x, s4z, s4vx, s4vz)

        x[alive_indices] = state_x + dt / 6.0 * (k1x + 2.0 * k2x + 2.0 * k3x + k4x)
        z[alive_indices] = state_z + dt / 6.0 * (k1z + 2.0 * k2z + 2.0 * k3z + k4z)
        vx[alive_indices] = state_vx + dt / 6.0 * (k1vx + 2.0 * k2vx + 2.0 * k3vx + k4vx)
        vz[alive_indices] = state_vz + dt / 6.0 * (k1vz + 2.0 * k2vz + 2.0 * k3vz + k4vz)

        current_time += dt

        crossed = alive_indices[x[alive_indices] >= target_distance_m[alive_indices]]
        if crossed.size:
            prev_x = x[crossed] - vx[crossed] * dt
            prev_z = z[crossed] - vz[crossed] * dt
            denominator = x[crossed] - prev_x
            denominator = np.where(np.abs(denominator) < 1e-9, 1e-9, denominator)
            fraction = (target_distance_m[crossed] - prev_x) / denominator
            z_at_target[crossed] = prev_z + fraction * (z[crossed] - prev_z)
            tof_at_target[crossed] = current_time - dt + fraction * dt
            reached[crossed] = True
            alive[crossed] = False

        grounded = alive_indices[z[alive_indices] < 0.0]
        if grounded.size:
            alive[grounded] = False

        unstable = alive_indices[
            ~np.isfinite(x[alive_indices])
            | ~np.isfinite(z[alive_indices])
            | ~np.isfinite(vx[alive_indices])
            | ~np.isfinite(vz[alive_indices])
            | (np.abs(vx[alive_indices]) > max_speed_mps)
            | (np.abs(vz[alive_indices]) > max_speed_mps)
        ]
        if unstable.size:
            alive[unstable] = False

    return z_at_target, tof_at_target, reached


def _simulate_entry_crossings_batch(
    exit_velocity_mps: np.ndarray,
    k_drag: np.ndarray,
    k_magnus: np.ndarray,
    launch_angle_deg: float,
    *,
    dt: float = DEFAULT_DT,
    max_time: float = DEFAULT_MAX_SIM_TIME,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    max_speed_mps = 250.0
    theta = math.radians(launch_angle_deg)
    vx = exit_velocity_mps * math.cos(theta)
    vz = exit_velocity_mps * math.sin(theta)
    x = np.zeros_like(exit_velocity_mps, dtype=np.float64)
    z = np.full_like(exit_velocity_mps, EXIT_HEIGHT_M, dtype=np.float64)

    reached = np.zeros_like(exit_velocity_mps, dtype=bool)
    x_at_entry = np.full_like(exit_velocity_mps, np.nan, dtype=np.float64)
    tof_at_entry = np.full_like(exit_velocity_mps, np.nan, dtype=np.float64)
    alive = np.ones_like(exit_velocity_mps, dtype=bool)

    def derivatives(
        state_x: np.ndarray,
        state_z: np.ndarray,
        state_vx: np.ndarray,
        state_vz: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        safe_vx = np.nan_to_num(state_vx, nan=0.0, posinf=0.0, neginf=0.0)
        safe_vz = np.nan_to_num(state_vz, nan=0.0, posinf=0.0, neginf=0.0)
        speed = np.nan_to_num(np.hypot(safe_vx, safe_vz), nan=0.0, posinf=max_speed_mps, neginf=0.0)
        speed = np.minimum(speed, max_speed_mps)
        ax = -k_drag_alive * speed * safe_vx
        az = -9.81 - k_drag_alive * speed * safe_vz + k_magnus_alive * speed * speed
        return state_vx, state_vz, ax, az

    max_steps = int(max_time / dt)
    current_time = 0.0

    for _ in range(max_steps):
        if not np.any(alive):
            break

        alive_indices = np.flatnonzero(alive)
        k_drag_alive = k_drag[alive_indices]
        k_magnus_alive = k_magnus[alive_indices]

        state_x = x[alive_indices]
        state_z = z[alive_indices]
        state_vx = vx[alive_indices]
        state_vz = vz[alive_indices]

        prev_x = state_x.copy()
        prev_z = state_z.copy()

        k1x, k1z, k1vx, k1vz = derivatives(state_x, state_z, state_vx, state_vz)
        s2x = state_x + k1x * dt / 2.0
        s2z = state_z + k1z * dt / 2.0
        s2vx = state_vx + k1vx * dt / 2.0
        s2vz = state_vz + k1vz * dt / 2.0

        k2x, k2z, k2vx, k2vz = derivatives(s2x, s2z, s2vx, s2vz)
        s3x = state_x + k2x * dt / 2.0
        s3z = state_z + k2z * dt / 2.0
        s3vx = state_vx + k2vx * dt / 2.0
        s3vz = state_vz + k2vz * dt / 2.0

        k3x, k3z, k3vx, k3vz = derivatives(s3x, s3z, s3vx, s3vz)
        s4x = state_x + k3x * dt
        s4z = state_z + k3z * dt
        s4vx = state_vx + k3vx * dt
        s4vz = state_vz + k3vz * dt

        k4x, k4z, k4vx, k4vz = derivatives(s4x, s4z, s4vx, s4vz)

        x[alive_indices] = state_x + dt / 6.0 * (k1x + 2.0 * k2x + 2.0 * k3x + k4x)
        z[alive_indices] = state_z + dt / 6.0 * (k1z + 2.0 * k2z + 2.0 * k3z + k4z)
        vx[alive_indices] = state_vx + dt / 6.0 * (k1vx + 2.0 * k2vx + 2.0 * k3vx + k4vx)
        vz[alive_indices] = state_vz + dt / 6.0 * (k1vz + 2.0 * k2vz + 2.0 * k3vz + k4vz)

        current_time += dt

        crossed_mask = (prev_z > TARGET_HEIGHT_M) & (z[alive_indices] <= TARGET_HEIGHT_M)
        crossed = alive_indices[crossed_mask]
        if crossed.size:
            denominator = z[crossed] - prev_z[crossed_mask]
            denominator = np.where(np.abs(denominator) < 1e-9, -1e-9, denominator)
            fraction = (TARGET_HEIGHT_M - prev_z[crossed_mask]) / denominator
            x_at_entry[crossed] = prev_x[crossed_mask] + fraction * (x[crossed] - prev_x[crossed_mask])
            tof_at_entry[crossed] = current_time - dt + fraction * dt
            reached[crossed] = True
            alive[crossed] = False

        grounded = alive_indices[z[alive_indices] < 0.0]
        if grounded.size:
            alive[grounded] = False

        unstable = alive_indices[
            ~np.isfinite(x[alive_indices])
            | ~np.isfinite(z[alive_indices])
            | ~np.isfinite(vx[alive_indices])
            | ~np.isfinite(vz[alive_indices])
            | (np.abs(vx[alive_indices]) > max_speed_mps)
            | (np.abs(vz[alive_indices]) > max_speed_mps)
        ]
        if unstable.size:
            alive[unstable] = False

    return x_at_entry, tof_at_entry, reached


def _best_monotonic_video_match(
    measured_tofs: Sequence[float],
    predicted_tofs: Sequence[float],
) -> tuple[float, tuple[int, ...], tuple[int, ...]]:
    video_count = len(measured_tofs)
    distance_count = len(predicted_tofs)

    if video_count > distance_count:
        return float("inf"), (), ()

    squared_error = [[0.0] * distance_count for _ in range(video_count)]
    for video_index, measured in enumerate(measured_tofs):
        for distance_index, predicted in enumerate(predicted_tofs):
            if not math.isfinite(predicted) or predicted <= 0.0:
                squared_error[video_index][distance_index] = float("inf")
                continue
            relative_error = (predicted - measured) / measured
            squared_error[video_index][distance_index] = relative_error * relative_error

    dp = [[float("inf")] * (distance_count + 1) for _ in range(video_count + 1)]
    take = [[False] * (distance_count + 1) for _ in range(video_count + 1)]
    for distance_index in range(distance_count + 1):
        dp[0][distance_index] = 0.0

    for video_index in range(1, video_count + 1):
        for distance_index in range(1, distance_count + 1):
            skip_cost = dp[video_index][distance_index - 1]
            use_cost = dp[video_index - 1][distance_index - 1] + squared_error[video_index - 1][distance_index - 1]
            if use_cost < skip_cost:
                dp[video_index][distance_index] = use_cost
                take[video_index][distance_index] = True
            else:
                dp[video_index][distance_index] = skip_cost

    if not math.isfinite(dp[video_count][distance_count]):
        return float("inf"), (), ()

    matched_indices = [0] * video_count
    video_index = video_count
    distance_index = distance_count
    used_distances = set()
    while video_index > 0 and distance_index > 0:
        if take[video_index][distance_index]:
            matched_indices[video_index - 1] = distance_index - 1
            used_distances.add(distance_index - 1)
            video_index -= 1
            distance_index -= 1
        else:
            distance_index -= 1

    unmatched_indices = tuple(index for index in range(distance_count) if index not in used_distances)
    rms_relative_error = math.sqrt(dp[video_count][distance_count] / video_count)
    return rms_relative_error, tuple(matched_indices), unmatched_indices


def _candidate_loss(
    empirical_distances: np.ndarray,
    empirical_rps: np.ndarray,
    measured_video_tofs: Sequence[float],
    *,
    predicted_rps: Sequence[float],
    predicted_tofs: Sequence[float],
    reachable: Sequence[bool],
    objective_distances: Sequence[float] | None = None,
    objective_rps: Sequence[float] | None = None,
    objective_tofs: Sequence[float] | None = None,
    candidate: FitCandidate | None = None,
) -> tuple[float, float, float, tuple[int, ...], tuple[int, ...]]:
    if not all(reachable):
        return float("inf"), float("inf"), float("inf"), (), ()

    predicted_rps_array = np.asarray(predicted_rps, dtype=np.float64)
    predicted_tofs_array = np.asarray(predicted_tofs, dtype=np.float64)
    if np.any(~np.isfinite(predicted_rps_array)) or np.any(~np.isfinite(predicted_tofs_array)):
        return float("inf"), float("inf"), float("inf"), (), ()

    if np.any(np.diff(predicted_tofs_array) <= 0.0):
        return float("inf"), float("inf"), float("inf"), (), ()

    rps_rms_relative = float(
        math.sqrt(
            float(
                np.mean(
                    np.square((predicted_rps_array - empirical_rps) / empirical_rps)
                )
            )
        )
    )
    tof_rms_relative, matched_indices, unmatched_indices = _best_monotonic_video_match(
        measured_video_tofs,
        predicted_tofs_array.tolist(),
    )
    if not math.isfinite(tof_rms_relative):
        return float("inf"), float("inf"), float("inf"), (), ()

    distance_penalty = 0.0
    if empirical_distances[0] > LUT_START_M or empirical_distances[-1] < 4.0:
        distance_penalty = 0.05

    smoothness_penalty = 0.0
    if objective_distances is not None and objective_rps is not None and objective_tofs is not None:
        _, _, _, _, smoothness_penalty = _smoothness_stats(
            objective_distances,
            objective_rps,
            objective_tofs,
        )

    edge_penalty = 0.0 if candidate is None else _parameter_edge_penalty(candidate)
    total_loss = rps_rms_relative + tof_rms_relative + smoothness_penalty + edge_penalty + distance_penalty
    return total_loss, rps_rms_relative, tof_rms_relative, matched_indices, unmatched_indices


def _coarse_proxy_search(
    empirical_points: Sequence[CalibrationPoint],
    video_summaries: Sequence[VideoSummary],
    launch_angle_deg: float,
    deadline_sec: float | None = None,
) -> FitCandidate:
    empirical_distances = np.array([point.center_distance_m for point in empirical_points], dtype=np.float64)
    empirical_rps = np.array([point.flywheel_rps for point in empirical_points], dtype=np.float64)
    measured_video_tofs = [summary.median_tof_sec for summary in video_summaries]

    anchor_slips = np.linspace(SLIP_MIN, SLIP_MAX, 5, dtype=np.float64)
    anchor_drags = np.linspace(DRAG_MIN, DRAG_MAX, 5, dtype=np.float64)
    anchor_magnus = np.linspace(MAGNUS_MIN, MAGNUS_MAX, 7, dtype=np.float64)
    anchor_candidates = np.array(
        list(itertools.product(anchor_slips, anchor_drags, anchor_magnus)),
        dtype=np.float64,
    )

    rng = np.random.default_rng(20260326 + int(round(launch_angle_deg * 1000.0)))
    random_candidates = np.column_stack(
        (
            rng.uniform(SLIP_MIN, SLIP_MAX, PROXY_GLOBAL_SAMPLES_PER_ANGLE),
            rng.uniform(DRAG_MIN, DRAG_MAX, PROXY_GLOBAL_SAMPLES_PER_ANGLE),
            rng.uniform(MAGNUS_MIN, MAGNUS_MAX, PROXY_GLOBAL_SAMPLES_PER_ANGLE),
        )
    )
    candidates = np.vstack((anchor_candidates, random_candidates))

    best_candidate: FitCandidate | None = None
    points_per_candidate = empirical_distances.size
    batch_size = 2048

    for batch_start in range(0, len(candidates), batch_size):
        if deadline_sec is not None and time.monotonic() >= deadline_sec and best_candidate is not None:
            break
        batch_candidates = candidates[batch_start: batch_start + batch_size]
        slips = batch_candidates[:, 0]
        drags = batch_candidates[:, 1]
        magnus = batch_candidates[:, 2]

        exit_velocity = np.repeat(slips, points_per_candidate) * np.tile(
            empirical_rps * math.pi * WHEEL_DIAMETER_M,
            len(batch_candidates),
        )
        target_distance = np.tile(empirical_distances, len(batch_candidates))
        z_at_target, tof_at_target, reached = _simulate_batch(
            exit_velocity_mps=exit_velocity,
            target_distance_m=target_distance,
            k_drag=np.repeat(_k_drag(drags), points_per_candidate),
            k_magnus=np.repeat(_k_magnus(magnus), points_per_candidate),
            launch_angle_deg=launch_angle_deg,
        )

        z_at_target = z_at_target.reshape(len(batch_candidates), points_per_candidate)
        tof_at_target = tof_at_target.reshape(len(batch_candidates), points_per_candidate)
        reached = reached.reshape(len(batch_candidates), points_per_candidate)
        _, _, long_range_reached = _simulate_batch(
            exit_velocity_mps=slips * TARGET_RPS_MAX * math.pi * WHEEL_DIAMETER_M,
            target_distance_m=np.full(len(batch_candidates), LUT_END_M, dtype=np.float64),
            k_drag=_k_drag(drags),
            k_magnus=_k_magnus(magnus),
            launch_angle_deg=launch_angle_deg,
        )

        for candidate_index, (slip, drag, magnus_coeff) in enumerate(batch_candidates.tolist()):
            if not np.all(reached[candidate_index]):
                continue
            if not bool(long_range_reached[candidate_index]):
                continue
            height_error = z_at_target[candidate_index] - TARGET_HEIGHT_M
            height_rms = float(math.sqrt(float(np.mean(np.square(height_error / HEIGHT_TOLERANCE_M)))))
            tof_rms_relative, matched_indices, unmatched_indices = _best_monotonic_video_match(
                measured_video_tofs,
                tof_at_target[candidate_index].tolist(),
            )
            if not math.isfinite(tof_rms_relative):
                continue

            coarse_candidate = FitCandidate(
                slip_factor=float(slip),
                drag_coeff=float(drag),
                magnus_coeff=float(magnus_coeff),
                launch_angle_deg=float(launch_angle_deg),
                loss=0.0,
                rps_rms_relative=height_rms,
                tof_rms_relative=tof_rms_relative,
                matched_video_indices=matched_indices,
                unmatched_distance_indices=unmatched_indices,
            )
            total_loss = height_rms + tof_rms_relative + _parameter_edge_penalty(coarse_candidate)
            if best_candidate is None or total_loss < best_candidate.loss:
                best_candidate = dataclasses.replace(coarse_candidate, loss=total_loss)

    if best_candidate is None:
        raise ValueError("Coarse proxy search did not find a reachable candidate")

    return best_candidate


def _local_grid(center: float, step: float, radius_steps: int, low: float, high: float) -> list[float]:
    values = {
        round(min(high, max(low, center + offset * step)), 6)
        for offset in range(-radius_steps, radius_steps + 1)
    }
    return sorted(values)


def _solve_exact_candidate_batch(
    slip_values: np.ndarray,
    drag_values: np.ndarray,
    magnus_values: np.ndarray,
    launch_angle_deg: float,
    distances_m: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    candidate_count = slip_values.size
    distance_count = distances_m.size

    slip_repeat = np.repeat(slip_values, distance_count)
    drag_repeat = np.repeat(drag_values, distance_count)
    magnus_repeat = np.repeat(magnus_values, distance_count)
    target_distance = np.tile(distances_m, candidate_count)

    min_exit_velocity = slip_repeat * TARGET_RPS_MIN * math.pi * WHEEL_DIAMETER_M
    max_exit_velocity = slip_repeat * TARGET_RPS_MAX * math.pi * WHEEL_DIAMETER_M
    z_at_high, tof_at_high, reached_at_high = _simulate_batch(
        exit_velocity_mps=max_exit_velocity,
        target_distance_m=target_distance,
        k_drag=_k_drag(drag_repeat),
        k_magnus=_k_magnus(magnus_repeat),
        launch_angle_deg=launch_angle_deg,
    )

    best_exit_velocity = max_exit_velocity.copy()
    best_tof = tof_at_high.copy()
    best_error = np.abs(z_at_high - TARGET_HEIGHT_M)
    reachable = reached_at_high.copy()
    reachable &= np.isfinite(best_error)

    low = min_exit_velocity.copy()
    high = max_exit_velocity.copy()
    done = np.zeros_like(max_exit_velocity, dtype=bool)
    done |= ~reached_at_high

    for _ in range(BINARY_SEARCH_ITERS):
        active = ~done
        if not np.any(active):
            break

        mid = (low[active] + high[active]) / 2.0
        z_mid, tof_mid, reached_mid = _simulate_batch(
            exit_velocity_mps=mid,
            target_distance_m=target_distance[active],
            k_drag=_k_drag(drag_repeat[active]),
            k_magnus=_k_magnus(magnus_repeat[active]),
            launch_angle_deg=launch_angle_deg,
        )

        active_indices = np.flatnonzero(active)
        unresolved = active_indices[~reached_mid]
        if unresolved.size:
            low[unresolved] = mid[~reached_mid]

        resolved = active_indices[reached_mid]
        if not resolved.size:
            continue
        error = z_mid[reached_mid] - TARGET_HEIGHT_M
        abs_error = np.abs(error)
        improved = abs_error < best_error[resolved]
        if np.any(improved):
            improved_indices = resolved[improved]
            best_error[improved_indices] = abs_error[improved]
            best_exit_velocity[improved_indices] = mid[reached_mid][improved]
            best_tof[improved_indices] = tof_mid[reached_mid][improved]

        converged = abs_error < HEIGHT_TOLERANCE_M
        if np.any(converged):
            converged_indices = resolved[converged]
            best_error[converged_indices] = abs_error[converged]
            best_exit_velocity[converged_indices] = mid[reached_mid][converged]
            best_tof[converged_indices] = tof_mid[reached_mid][converged]
            done[converged_indices] = True

        too_high = error > 0.0
        if np.any(too_high):
            high[resolved[too_high]] = mid[reached_mid][too_high]
        if np.any(~too_high):
            low[resolved[~too_high]] = mid[reached_mid][~too_high]

    reachable &= best_error < REACHABLE_HEIGHT_TOLERANCE_M
    predicted_rps = best_exit_velocity / (math.pi * WHEEL_DIAMETER_M * slip_repeat)

    return (
        predicted_rps.reshape(candidate_count, distance_count),
        best_tof.reshape(candidate_count, distance_count),
        reachable.reshape(candidate_count, distance_count),
    )


def _exact_local_search(
    empirical_points: Sequence[CalibrationPoint],
    video_summaries: Sequence[VideoSummary],
    coarse_candidate: FitCandidate,
    step: float,
    radius_steps: int,
    deadline_sec: float | None = None,
) -> FitCandidate:
    empirical_distances = np.array([point.center_distance_m for point in empirical_points], dtype=np.float64)
    empirical_rps = np.array([point.flywheel_rps for point in empirical_points], dtype=np.float64)
    measured_video_tofs = [summary.median_tof_sec for summary in video_summaries]

    slip_values = _local_grid(coarse_candidate.slip_factor, step, radius_steps, SLIP_MIN, SLIP_MAX)
    drag_values = _local_grid(coarse_candidate.drag_coeff, step, radius_steps, DRAG_MIN, DRAG_MAX)
    magnus_values = _local_grid(coarse_candidate.magnus_coeff, step, radius_steps, MAGNUS_MIN, MAGNUS_MAX)
    candidates = list(itertools.product(slip_values, drag_values, magnus_values))

    best_candidate: FitCandidate | None = None
    batch_size = 1024

    for batch_start in range(0, len(candidates), batch_size):
        if deadline_sec is not None and time.monotonic() >= deadline_sec and best_candidate is not None:
            break
        batch_candidates = candidates[batch_start: batch_start + batch_size]
        slip_batch = np.array([candidate[0] for candidate in batch_candidates], dtype=np.float64)
        drag_batch = np.array([candidate[1] for candidate in batch_candidates], dtype=np.float64)
        magnus_batch = np.array([candidate[2] for candidate in batch_candidates], dtype=np.float64)
        predicted_rps, predicted_tofs, reachable = _solve_exact_candidate_batch(
            slip_values=slip_batch,
            drag_values=drag_batch,
            magnus_values=magnus_batch,
            launch_angle_deg=coarse_candidate.launch_angle_deg,
            distances_m=empirical_distances,
        )
        objective_sample_rps, objective_sample_tofs, objective_sample_reachable = _solve_exact_candidate_batch(
            slip_values=slip_batch,
            drag_values=drag_batch,
            magnus_values=magnus_batch,
            launch_angle_deg=coarse_candidate.launch_angle_deg,
            distances_m=OBJECTIVE_SAMPLE_DISTANCES,
        )
        _, _, lut_range_reachable = _solve_exact_candidate_batch(
            slip_values=slip_batch,
            drag_values=drag_batch,
            magnus_values=magnus_batch,
            launch_angle_deg=coarse_candidate.launch_angle_deg,
            distances_m=np.array([LUT_END_M], dtype=np.float64),
        )

        for candidate_index, (slip, drag, magnus_coeff) in enumerate(batch_candidates):
            if not bool(lut_range_reachable[candidate_index][0]):
                continue
            if not np.all(objective_sample_reachable[candidate_index]):
                continue
            current_candidate = FitCandidate(
                slip_factor=float(slip),
                drag_coeff=float(drag),
                magnus_coeff=float(magnus_coeff),
                launch_angle_deg=coarse_candidate.launch_angle_deg,
                loss=0.0,
                rps_rms_relative=0.0,
                tof_rms_relative=0.0,
                matched_video_indices=(),
                unmatched_distance_indices=(),
            )
            loss, rps_rms_relative, tof_rms_relative, matched_indices, unmatched_indices = _candidate_loss(
                empirical_distances=empirical_distances,
                empirical_rps=empirical_rps,
                measured_video_tofs=measured_video_tofs,
                predicted_rps=predicted_rps[candidate_index],
                predicted_tofs=predicted_tofs[candidate_index],
                reachable=reachable[candidate_index],
                objective_distances=OBJECTIVE_SAMPLE_DISTANCES,
                objective_rps=objective_sample_rps[candidate_index],
                objective_tofs=objective_sample_tofs[candidate_index],
                candidate=current_candidate,
            )
            if not math.isfinite(loss):
                continue

            if best_candidate is None or loss < best_candidate.loss:
                best_candidate = dataclasses.replace(
                    current_candidate,
                    loss=float(loss),
                    rps_rms_relative=float(rps_rms_relative),
                    tof_rms_relative=float(tof_rms_relative),
                    matched_video_indices=matched_indices,
                    unmatched_distance_indices=unmatched_indices,
                )

    if best_candidate is None:
        raise ValueError("Exact local search did not find a valid candidate")

    return best_candidate


def _best_candidates(candidates: Iterable[FitCandidate], limit: int) -> list[FitCandidate]:
    return sorted(candidates, key=lambda candidate: candidate.loss)[:limit]


def _search_best_candidate(
    empirical_points: Sequence[CalibrationPoint],
    video_summaries: Sequence[VideoSummary],
    deadline_sec: float | None = None,
) -> FitCandidate:
    coarse_angle_candidates: list[FitCandidate] = []
    angle_values = np.arange(ANGLE_MIN_DEG, ANGLE_MAX_DEG + 1e-9, ANGLE_PROXY_STEP_DEG)
    for angle_deg in angle_values:
        if deadline_sec is not None and time.monotonic() >= deadline_sec and coarse_angle_candidates:
            break
        coarse_angle_candidates.append(
            _coarse_proxy_search(
                empirical_points,
                video_summaries,
                float(round(angle_deg, 3)),
                deadline_sec=deadline_sec,
            )
        )

    refined_candidates: list[FitCandidate] = []
    for candidate in _best_candidates(coarse_angle_candidates, GLOBAL_SEED_COUNT):
        if deadline_sec is not None and time.monotonic() >= deadline_sec and refined_candidates:
            break
        refined_candidates.append(
            _exact_local_search(
                empirical_points,
                video_summaries,
                candidate,
                REFINE_STEP,
                6,
                deadline_sec=deadline_sec,
            )
        )
    if not refined_candidates:
        refined_candidates = _best_candidates(coarse_angle_candidates, 1)

    angle_refined_candidates: list[FitCandidate] = []
    for candidate in _best_candidates(refined_candidates, LOCAL_REFINED_COUNT):
        if deadline_sec is not None and time.monotonic() >= deadline_sec and angle_refined_candidates:
            break
        local_angles = np.arange(
            candidate.launch_angle_deg - ANGLE_PROXY_STEP_DEG,
            candidate.launch_angle_deg + ANGLE_PROXY_STEP_DEG + 1e-9,
            ANGLE_REFINE_STEP_DEG,
        )
        for angle_deg in local_angles:
            if deadline_sec is not None and time.monotonic() >= deadline_sec and angle_refined_candidates:
                break
            angle_refined_candidates.append(
                _exact_local_search(
                    empirical_points,
                    video_summaries,
                    dataclasses.replace(candidate, launch_angle_deg=float(round(angle_deg, 3))),
                    REFINE_STEP,
                    4,
                    deadline_sec=deadline_sec,
                )
            )

    final_seed = _best_candidates(angle_refined_candidates or refined_candidates, 1)[0]
    return _exact_local_search(
        empirical_points,
        video_summaries,
        final_seed,
        FINAL_STEP,
        6,
        deadline_sec=deadline_sec,
    )


def format_lut_entries(entries: Sequence[tuple[float, float, float]]) -> str:
    return "\n".join(
        f"    ({distance:.12f}, {flywheel_rps:.12f}, {tof_sec:.12f}),"
        for distance, flywheel_rps, tof_sec in entries
    )


def generate_lut_entries(
    candidate: FitCandidate,
    empirical_points: Sequence[CalibrationPoint] | None = None,
    video_summaries: Sequence[VideoSummary] | None = None,
) -> list[tuple[float, float, float]]:
    grid_distances = np.round(np.arange(LUT_START_M, LUT_END_M + 1e-9, LUT_STEP_M), 2)
    empirical_distances = [] if empirical_points is None else [point.center_distance_m for point in empirical_points]
    distances = np.array(sorted({*grid_distances.tolist(), *empirical_distances}), dtype=np.float64)
    predicted_rps, predicted_tofs, reachable = _solve_exact_candidate_batch(
        slip_values=np.array([candidate.slip_factor], dtype=np.float64),
        drag_values=np.array([candidate.drag_coeff], dtype=np.float64),
        magnus_values=np.array([candidate.magnus_coeff], dtype=np.float64),
        launch_angle_deg=candidate.launch_angle_deg,
        distances_m=distances,
    )

    if not np.all(reachable[0]):
        unreachable_index = int(np.flatnonzero(~reachable[0])[0])
        raise ValueError(
            f"Calibrated model cannot reach LUT distance {distances[unreachable_index]:.2f} m"
        )

    empirical_rps_overrides = (
        {}
        if empirical_points is None
        else {point.center_distance_m: point.flywheel_rps for point in empirical_points}
    )

    return [
        (
            float(distance),
            float(empirical_rps_overrides.get(float(distance), rps)),
            float(tof),
        )
        for distance, rps, tof in zip(distances, predicted_rps[0], predicted_tofs[0], strict=True)
    ]


def _score_probabilities(
    candidate: FitCandidate,
    distances_m: np.ndarray,
    predicted_rps: np.ndarray,
    predicted_tofs: np.ndarray,
    rps_residuals: np.ndarray,
    tof_residuals: np.ndarray,
    *,
    sample_count: int = ROBUSTNESS_MONTE_CARLO_SAMPLES,
) -> tuple[np.ndarray, float, float, float]:
    rng = np.random.default_rng(20260326)

    if rps_residuals.size == 0:
        rps_residuals = np.array([0.0], dtype=np.float64)
    if tof_residuals.size == 0:
        tof_residuals = np.array([0.0], dtype=np.float64)

    sampled_rps_errors = rng.choice(rps_residuals, size=(distances_m.size, sample_count), replace=True)
    sampled_tof_errors = rng.choice(tof_residuals, size=(distances_m.size, sample_count), replace=True)

    effective_rps = np.clip(predicted_rps[:, None] - sampled_rps_errors, TARGET_RPS_MIN, TARGET_RPS_MAX)
    exit_velocity = candidate.slip_factor * effective_rps.reshape(-1) * math.pi * WHEEL_DIAMETER_M

    k_drag = np.full(exit_velocity.shape, float(_k_drag(np.array([candidate.drag_coeff]))[0]), dtype=np.float64)
    k_magnus = np.full(exit_velocity.shape, float(_k_magnus(np.array([candidate.magnus_coeff]))[0]), dtype=np.float64)
    x_at_entry, _, reached = _simulate_entry_crossings_batch(
        exit_velocity_mps=exit_velocity,
        k_drag=k_drag,
        k_magnus=k_magnus,
        launch_angle_deg=candidate.launch_angle_deg,
    )

    x_at_entry = x_at_entry.reshape(distances_m.size, sample_count)
    reached = reached.reshape(distances_m.size, sample_count)
    radial_miss = np.abs(x_at_entry - distances_m[:, None])

    actual_tofs = np.clip(predicted_tofs[:, None] - sampled_tof_errors, 0.05, DEFAULT_MAX_SIM_TIME)
    predicted_displacement = MAX_SOTM_SPEED_MPS * _drag_compensated_time(predicted_tofs[:, None])
    actual_displacement = MAX_SOTM_SPEED_MPS * _drag_compensated_time(actual_tofs)
    lateral_miss = np.abs(predicted_displacement - actual_displacement)

    total_miss = np.hypot(radial_miss, lateral_miss)
    score_mask = reached & np.isfinite(total_miss) & (total_miss <= USABLE_SCORING_RADIUS_M)
    probabilities = np.mean(score_mask, axis=1)

    return (
        probabilities,
        float(np.mean(probabilities)),
        float(np.min(probabilities)),
        float(np.percentile(probabilities, 10)),
    )


def _assess_candidate(
    candidate: FitCandidate,
    empirical_points: Sequence[CalibrationPoint],
    video_summaries: Sequence[VideoSummary],
    *,
    baseline_tof_rms_relative: float | None = None,
    baseline_empirical_score_mean: float | None = None,
    baseline_interpolated_score_mean: float | None = None,
    require_relative_improvement: bool = False,
) -> CandidateAssessment:
    empirical_distances = np.array([point.center_distance_m for point in empirical_points], dtype=np.float64)
    empirical_rps = np.array([point.flywheel_rps for point in empirical_points], dtype=np.float64)
    measured_video_tofs = [summary.median_tof_sec for summary in video_summaries]

    predicted_empirical_rps, predicted_empirical_tofs, reachable = _solve_exact_candidate_batch(
        slip_values=np.array([candidate.slip_factor], dtype=np.float64),
        drag_values=np.array([candidate.drag_coeff], dtype=np.float64),
        magnus_values=np.array([candidate.magnus_coeff], dtype=np.float64),
        launch_angle_deg=candidate.launch_angle_deg,
        distances_m=empirical_distances,
    )
    loss, rps_rms_relative, tof_rms_relative, matched_indices, unmatched_indices = _candidate_loss(
        empirical_distances=empirical_distances,
        empirical_rps=empirical_rps,
        measured_video_tofs=measured_video_tofs,
        predicted_rps=predicted_empirical_rps[0],
        predicted_tofs=predicted_empirical_tofs[0],
        reachable=reachable[0],
        candidate=candidate,
    )
    if not math.isfinite(loss):
        raise ValueError("Candidate assessment failed because the candidate is unreachable")

    lut_entries = tuple(generate_lut_entries(candidate, empirical_points, None))
    lut_distances = np.array([entry[0] for entry in lut_entries], dtype=np.float64)
    lut_rps = np.array([entry[1] for entry in lut_entries], dtype=np.float64)
    lut_tofs = np.array([entry[2] for entry in lut_entries], dtype=np.float64)
    max_adj_rps, max_adj_tof, rps_curvature_rms, tof_curvature_rms, _ = _smoothness_stats(
        lut_distances,
        lut_rps,
        lut_tofs,
    )

    matched_measured_tofs = np.array(
        [video_summaries[index].median_tof_sec for index in range(len(matched_indices))],
        dtype=np.float64,
    )
    matched_predicted_tofs = predicted_empirical_tofs[0][list(matched_indices)]
    tof_relative_errors = np.abs((matched_predicted_tofs - matched_measured_tofs) / matched_measured_tofs)
    rps_relative_errors = np.abs((predicted_empirical_rps[0] - empirical_rps) / empirical_rps)

    rps_residuals = predicted_empirical_rps[0] - empirical_rps
    tof_residuals = matched_predicted_tofs - matched_measured_tofs

    empirical_probabilities_array, empirical_score_mean, empirical_score_min, _ = _score_probabilities(
        candidate,
        empirical_distances,
        predicted_empirical_rps[0],
        predicted_empirical_tofs[0],
        rps_residuals,
        tof_residuals,
    )

    interpolated_distances = OBJECTIVE_SAMPLE_DISTANCES
    predicted_interpolated_rps, predicted_interpolated_tofs, interpolated_reachable = _solve_exact_candidate_batch(
        slip_values=np.array([candidate.slip_factor], dtype=np.float64),
        drag_values=np.array([candidate.drag_coeff], dtype=np.float64),
        magnus_values=np.array([candidate.magnus_coeff], dtype=np.float64),
        launch_angle_deg=candidate.launch_angle_deg,
        distances_m=interpolated_distances,
    )
    if not np.all(interpolated_reachable[0]):
        raise ValueError("Candidate interpolation assessment failed because a test distance is unreachable")

    interpolated_probabilities_array, interpolated_score_mean, interpolated_score_min, _ = _score_probabilities(
        candidate,
        interpolated_distances,
        predicted_interpolated_rps[0],
        predicted_interpolated_tofs[0],
        rps_residuals,
        tof_residuals,
    )

    reasons: list[str] = []
    accepted = True

    if rps_rms_relative > 0.015:
        accepted = False
        reasons.append(f"RPS RMS too high ({rps_rms_relative:.4f})")
    elif rps_rms_relative > 0.010:
        reasons.append(f"RPS RMS above target ({rps_rms_relative:.4f})")

    if baseline_tof_rms_relative is not None and tof_rms_relative >= baseline_tof_rms_relative:
        accepted = False
        reasons.append(
            f"TOF RMS did not beat baseline ({tof_rms_relative:.4f} >= {baseline_tof_rms_relative:.4f})"
        )

    if max_adj_tof > 0.03:
        accepted = False
        reasons.append(f"Max TOF jump too high ({max_adj_tof:.4f}s)")
    if max_adj_rps > 1.5:
        accepted = False
        reasons.append(f"Max RPS jump too high ({max_adj_rps:.4f} rps)")

    if empirical_score_mean < 0.90:
        accepted = False
        reasons.append(f"Empirical score probability mean too low ({empirical_score_mean:.3f})")
    if interpolated_score_mean < 0.80:
        accepted = False
        reasons.append(f"Interpolated score probability mean too low ({interpolated_score_mean:.3f})")

    overall_score_min = min(empirical_score_min, interpolated_score_min)
    if overall_score_min < 0.70:
        accepted = False
        reasons.append(f"Score probability floor too low ({overall_score_min:.3f})")

    if require_relative_improvement:
        if baseline_empirical_score_mean is not None and empirical_score_mean < baseline_empirical_score_mean:
            accepted = False
            reasons.append(
                f"Empirical score robustness did not beat baseline ({empirical_score_mean:.3f} < {baseline_empirical_score_mean:.3f})"
            )
        if (
            baseline_interpolated_score_mean is not None
            and interpolated_score_mean < baseline_interpolated_score_mean
        ):
            accepted = False
            reasons.append(
                f"Interpolated score robustness did not beat baseline ({interpolated_score_mean:.3f} < {baseline_interpolated_score_mean:.3f})"
            )

    tof_median_relative = float(np.median(tof_relative_errors))
    tof_over_40 = int(np.count_nonzero(tof_relative_errors > 0.40))
    tof_over_60 = int(np.count_nonzero(tof_relative_errors > 0.60))
    if tof_median_relative > 0.20:
        accepted = False
        reasons.append(f"Median matched TOF relative error too high ({tof_median_relative:.3f})")
    if float(np.percentile(tof_relative_errors, 90)) > 0.40:
        accepted = False
        reasons.append(f"90th percentile matched TOF relative error too high ({np.percentile(tof_relative_errors, 90):.3f})")
    if tof_over_40 > 2:
        accepted = False
        reasons.append(f"Too many matched TOF points exceed 40% error ({tof_over_40})")
    if tof_over_60 > 0:
        accepted = False
        reasons.append(f"Matched TOF points exceed 60% error ({tof_over_60})")

    return CandidateAssessment(
        candidate=dataclasses.replace(
            candidate,
            loss=loss,
            rps_rms_relative=rps_rms_relative,
            tof_rms_relative=tof_rms_relative,
            matched_video_indices=matched_indices,
            unmatched_distance_indices=unmatched_indices,
        ),
        rps_rms_relative=rps_rms_relative,
        tof_rms_relative=tof_rms_relative,
        max_adj_rps=max_adj_rps,
        max_adj_tof=max_adj_tof,
        rps_curvature_rms=rps_curvature_rms,
        tof_curvature_rms=tof_curvature_rms,
        empirical_score_mean=empirical_score_mean,
        empirical_score_min=empirical_score_min,
        interpolated_score_mean=interpolated_score_mean,
        interpolated_score_min=interpolated_score_min,
        overall_score_min=overall_score_min,
        rps_relative_p90=float(np.percentile(rps_relative_errors, 90)),
        tof_relative_p90=float(np.percentile(tof_relative_errors, 90)),
        matched_video_indices=matched_indices,
        unmatched_distance_indices=unmatched_indices,
        accepted=accepted,
        reasons=tuple(reasons),
        lut_entries=lut_entries,
        empirical_probabilities=tuple(
            (float(distance), float(probability))
            for distance, probability in zip(empirical_distances, empirical_probabilities_array, strict=True)
        ),
        interpolated_probabilities=tuple(
            (float(distance), float(probability))
            for distance, probability in zip(interpolated_distances, interpolated_probabilities_array, strict=True)
        ),
    )


def _find_wpilib_java_tools() -> tuple[Path, Path] | None:
    candidates = [
        (
            Path.home() / "wpilib" / "2026" / "jdk" / "bin" / "javac",
            Path.home() / "wpilib" / "2026" / "jdk" / "bin" / "java",
        ),
        (
            Path("/usr/bin/javac"),
            Path("/usr/bin/java"),
        ),
    ]
    for javac_path, java_path in candidates:
        if javac_path.exists() and java_path.exists():
            return javac_path, java_path
    return None


def verify_python_against_java(candidate: FitCandidate) -> JavaVerification | None:
    java_tools = _find_wpilib_java_tools()
    if java_tools is None:
        return None

    javac_path, java_path = java_tools
    projectile_simulator_path = ROOT / "constants" / "shooter_on_the_move" / "frc-fire-control-main" / "src" / "main" / "java" / "frc" / "firecontrol" / "ProjectileSimulator.java"
    distances = [1.0, 2.0, 3.0, 4.0, 5.0]

    with tempfile.TemporaryDirectory(prefix="projectile-calibration-") as temp_dir:
        temp_root = Path(temp_dir)
        runner_path = temp_root / "ProjectileSimulatorVerifier.java"
        classes_dir = temp_root / "classes"
        classes_dir.mkdir(parents=True, exist_ok=True)
        runner_path.write_text(
            (
                "import frc.firecontrol.ProjectileSimulator;\n"
                "public class ProjectileSimulatorVerifier {\n"
                "  public static void main(String[] args) {\n"
                "    ProjectileSimulator.SimParameters params = new ProjectileSimulator.SimParameters(\n"
                f"        {BALL_MASS_KG}, {BALL_DIAMETER_M}, {candidate.drag_coeff}, {candidate.magnus_coeff}, {AIR_DENSITY},\n"
                f"        {EXIT_HEIGHT_M}, {WHEEL_DIAMETER_M}, {TARGET_HEIGHT_M}, {candidate.slip_factor}, {candidate.launch_angle_deg},\n"
                f"        {DEFAULT_DT}, {TARGET_RPM_MIN}, {TARGET_RPM_MAX}, {BINARY_SEARCH_ITERS}, {DEFAULT_MAX_SIM_TIME});\n"
                "    ProjectileSimulator sim = new ProjectileSimulator(params);\n"
                "    double[] distances = new double[] {1.0, 2.0, 3.0, 4.0, 5.0};\n"
                "    for (double distance : distances) {\n"
                "      ProjectileSimulator.LUTEntry entry = sim.findRPMForDistance(distance);\n"
                "      System.out.printf(\"%.3f,%.9f,%.9f,%s%n\", distance, entry.rpm(), entry.tof(), entry.reachable());\n"
                "    }\n"
                "  }\n"
                "}\n"
            ),
            encoding="utf-8",
        )

        subprocess.run(
            [
                str(javac_path),
                "-d",
                str(classes_dir),
                str(projectile_simulator_path),
                str(runner_path),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        completed = subprocess.run(
            [
                str(java_path),
                "-cp",
                str(classes_dir),
                "ProjectileSimulatorVerifier",
            ],
            check=True,
            capture_output=True,
            text=True,
        )

    python_rps, python_tofs, python_reachable = _solve_exact_candidate_batch(
        slip_values=np.array([candidate.slip_factor], dtype=np.float64),
        drag_values=np.array([candidate.drag_coeff], dtype=np.float64),
        magnus_values=np.array([candidate.magnus_coeff], dtype=np.float64),
        launch_angle_deg=candidate.launch_angle_deg,
        distances_m=np.array(distances, dtype=np.float64),
    )

    cases = []
    max_rpm_abs_error = 0.0
    max_tof_abs_error = 0.0
    for line_index, line in enumerate(completed.stdout.strip().splitlines()):
        distance_text, rpm_text, tof_text, reachable_text = line.split(",")
        distance = float(distance_text)
        java_rpm = float(rpm_text)
        java_tof = float(tof_text)
        java_reachable = reachable_text == "true"
        python_rpm = float(python_rps[0][line_index] * 60.0)
        python_tof = float(python_tofs[0][line_index])
        python_reach = bool(python_reachable[0][line_index])
        if java_reachable != python_reach:
            raise ValueError(f"Python/Java reachable mismatch at {distance:.2f} m")

        rpm_abs_error = abs(java_rpm - python_rpm)
        tof_abs_error = abs(java_tof - python_tof)
        max_rpm_abs_error = max(max_rpm_abs_error, rpm_abs_error)
        max_tof_abs_error = max(max_tof_abs_error, tof_abs_error)
        cases.append((distance, java_rpm, python_rpm, java_tof, python_tof))

    return JavaVerification(
        max_rpm_abs_error=max_rpm_abs_error,
        max_tof_abs_error=max_tof_abs_error,
        cases=tuple(cases),
    )


def build_report(
    empirical_points: Sequence[CalibrationPoint],
    video_summaries: Sequence[VideoSummary],
    baseline_assessment: CandidateAssessment,
    candidate_assessment: CandidateAssessment | None,
    chosen_assessment: CandidateAssessment,
    promoted: bool,
    runtime_sec: float,
    java_verification: JavaVerification | None,
) -> dict[str, object]:
    def serialize_assessment(assessment: CandidateAssessment) -> dict[str, object]:
        matched_assignments = [
            {
                "video_name": video_summaries[video_index].video_name,
                "matched_distance_m": empirical_points[distance_index].center_distance_m,
                "measured_tof_sec": video_summaries[video_index].median_tof_sec,
            }
            for video_index, distance_index in enumerate(assessment.matched_video_indices)
            if video_index < len(video_summaries) and distance_index < len(empirical_points)
        ]
        return {
            "fit": dataclasses.asdict(assessment.candidate),
            "accepted": assessment.accepted,
            "reasons": list(assessment.reasons),
            "metrics": {
                "rps_rms_relative": assessment.rps_rms_relative,
                "tof_rms_relative": assessment.tof_rms_relative,
                "max_adj_rps": assessment.max_adj_rps,
                "max_adj_tof": assessment.max_adj_tof,
                "rps_curvature_rms": assessment.rps_curvature_rms,
                "tof_curvature_rms": assessment.tof_curvature_rms,
                "rps_relative_p90": assessment.rps_relative_p90,
                "tof_relative_p90": assessment.tof_relative_p90,
                "empirical_score_mean": assessment.empirical_score_mean,
                "empirical_score_min": assessment.empirical_score_min,
                "interpolated_score_mean": assessment.interpolated_score_mean,
                "interpolated_score_min": assessment.interpolated_score_min,
                "overall_score_min": assessment.overall_score_min,
            },
            "matched_assignments": matched_assignments,
            "unmatched_distance_indices": list(assessment.unmatched_distance_indices),
            "empirical_probabilities": [
                {"distance_m": distance, "score_probability": probability}
                for distance, probability in assessment.empirical_probabilities
            ],
            "interpolated_probabilities": [
                {"distance_m": distance, "score_probability": probability}
                for distance, probability in assessment.interpolated_probabilities
            ],
            "lut_entry_count": len(assessment.lut_entries),
            "lut_preview": {
                "first": assessment.lut_entries[:5],
                "last": assessment.lut_entries[-5:],
            },
        }

    return {
        "hub_front_to_center_in": HUB_FRONT_TO_CENTER_IN,
        "capture_fps_used": CAPTURE_FPS,
        "max_sotm_speed_mps": MAX_SOTM_SPEED_MPS,
        "usable_scoring_radius_m": USABLE_SCORING_RADIUS_M,
        "runtime_sec": runtime_sec,
        "promoted_candidate": promoted,
        "chosen_fit": dataclasses.asdict(chosen_assessment.candidate),
        "csv_points": [dataclasses.asdict(point) for point in empirical_points],
        "video_summaries": [dataclasses.asdict(summary) for summary in video_summaries],
        "baseline": serialize_assessment(baseline_assessment),
        "candidate": None if candidate_assessment is None else serialize_assessment(candidate_assessment),
        "chosen": serialize_assessment(chosen_assessment),
        "java_verification": None if java_verification is None else dataclasses.asdict(java_verification),
    }


def main() -> None:
    args = parse_args()
    start_time = time.monotonic()
    overall_deadline = start_time + args.max_runtime_minutes * 60.0
    video_paths = tuple(args.videos) if args.videos else VIDEO_DEFAULTS

    empirical_points = load_empirical_points(args.csv)
    video_summaries = load_video_summaries(video_paths)
    baseline_assessment = _assess_candidate(_baseline_candidate(), empirical_points, video_summaries)

    reserve_seconds = 7.0 * 60.0
    max_search_window = 18.0 * 60.0
    search_deadline = min(overall_deadline - reserve_seconds, time.monotonic() + max_search_window)
    candidate_assessment: CandidateAssessment | None = None

    try:
        candidate = _search_best_candidate(
            empirical_points,
            video_summaries,
            deadline_sec=search_deadline,
        )
        candidate_assessment = _assess_candidate(
            candidate,
            empirical_points,
            video_summaries,
            baseline_tof_rms_relative=baseline_assessment.tof_rms_relative,
            baseline_empirical_score_mean=baseline_assessment.empirical_score_mean,
            baseline_interpolated_score_mean=baseline_assessment.interpolated_score_mean,
            require_relative_improvement=True,
        )
    except Exception as exc:
        candidate_assessment = None
        candidate_failure_reason = str(exc)
    else:
        candidate_failure_reason = None

    if candidate_assessment is not None and candidate_assessment.accepted:
        if candidate_assessment.max_adj_rps > baseline_assessment.max_adj_rps + 1e-9:
            candidate_assessment = dataclasses.replace(
                candidate_assessment,
                accepted=False,
                reasons=(
                    *candidate_assessment.reasons,
                    f"Max RPS jump regressed vs baseline ({candidate_assessment.max_adj_rps:.4f} > {baseline_assessment.max_adj_rps:.4f})",
                ),
            )
        if candidate_assessment.max_adj_tof > baseline_assessment.max_adj_tof + 1e-9:
            candidate_assessment = dataclasses.replace(
                candidate_assessment,
                accepted=False,
                reasons=(
                    *candidate_assessment.reasons,
                    f"Max TOF jump regressed vs baseline ({candidate_assessment.max_adj_tof:.4f} > {baseline_assessment.max_adj_tof:.4f})",
                ),
            )
        if candidate_assessment.rps_curvature_rms > baseline_assessment.rps_curvature_rms * 1.10:
            candidate_assessment = dataclasses.replace(
                candidate_assessment,
                accepted=False,
                reasons=(
                    *candidate_assessment.reasons,
                    f"RPS curvature regressed vs baseline ({candidate_assessment.rps_curvature_rms:.4f} > {baseline_assessment.rps_curvature_rms:.4f})",
                ),
            )
        if candidate_assessment.tof_curvature_rms > baseline_assessment.tof_curvature_rms * 1.10:
            candidate_assessment = dataclasses.replace(
                candidate_assessment,
                accepted=False,
                reasons=(
                    *candidate_assessment.reasons,
                    f"TOF curvature regressed vs baseline ({candidate_assessment.tof_curvature_rms:.4f} > {baseline_assessment.tof_curvature_rms:.4f})",
                ),
            )

    promoted = bool(candidate_assessment is not None and candidate_assessment.accepted)
    chosen_assessment = candidate_assessment if promoted else baseline_assessment
    java_verification = None if args.skip_java_check else verify_python_against_java(chosen_assessment.candidate)
    runtime_sec = time.monotonic() - start_time

    report = build_report(
        empirical_points=empirical_points,
        video_summaries=video_summaries,
        baseline_assessment=baseline_assessment,
        candidate_assessment=candidate_assessment,
        chosen_assessment=chosen_assessment,
        promoted=promoted,
        runtime_sec=runtime_sec,
        java_verification=java_verification,
    )
    if candidate_failure_reason is not None:
        report["candidate_failure_reason"] = candidate_failure_reason

    if args.json_out is not None:
        args.json_out.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2))
    print()
    print("# Chosen LUT Entries")
    print(format_lut_entries(chosen_assessment.lut_entries))


if __name__ == "__main__":
    main()
