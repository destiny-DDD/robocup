#!/usr/bin/env python3
"""Capture cropped white blocks for the A/B/C/D HOG+SVM dataset."""

import argparse
from pathlib import Path
import shutil
import subprocess

import cv2


def configure_camera(capture, exposure=None, brightness=None, focus=None,
                     v4l2_device=None):
    """Apply camera controls after opening it; unsupported controls are skipped."""
    if v4l2_device and shutil.which("v4l2-ctl"):
        if exposure is not None:
            commands = [
                ["v4l2-ctl", "-d", v4l2_device,
                 "--set-ctrl=auto_exposure=1"],
                ["v4l2-ctl", "-d", v4l2_device,
                 f"--set-ctrl=exposure_time_absolute={int(exposure)}"],
            ]
            for command in commands:
                result = subprocess.run(command, capture_output=True, text=True)
                if result.returncode != 0:
                    print(f"camera control failed: {' '.join(command)}")
                    if result.stderr:
                        print(result.stderr.strip())
        if brightness is not None:
            subprocess.run(
                ["v4l2-ctl", "-d", v4l2_device,
                 f"--set-ctrl=brightness={int(brightness)}"],
                check=False,
            )
        if focus is not None:
            subprocess.run(
                ["v4l2-ctl", "-d", v4l2_device,
                 "--set-ctrl=focus_automatic_continuous=0"],
                check=False,
            )
            subprocess.run(
                ["v4l2-ctl", "-d", v4l2_device,
                 f"--set-ctrl=focus_absolute={int(focus)}"],
                check=False,
            )
        return

    if exposure is not None:
        capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        capture.set(cv2.CAP_PROP_EXPOSURE, exposure)
    if brightness is not None:
        capture.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
    if focus is not None:
        capture.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        capture.set(cv2.CAP_PROP_FOCUS, focus)


def largest_white_block(frame, saturation_max, value_min):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv, (0, 0, value_min), (179, saturation_max, 255)
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)),
    )
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, width, height = cv2.boundingRect(contour)
        if area < 1500 or width < 10 or height < 10:
            continue
        fill = area / float(width * height)
        ratio = width / float(height)
        if fill >= 0.35 and 0.2 <= ratio <= 5.0:
            candidates.append((area, (x, y, width, height)))
    if not candidates:
        return None, mask
    _, (x, y, width, height) = max(candidates, key=lambda item: item[0])
    return (x, y, width, height), mask


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path, help="dataset directory")
    parser.add_argument("--camera", type=int, default=2)
    parser.add_argument("--saturation-max", type=int, default=60)
    parser.add_argument("--value-min", type=int, default=150)
    parser.add_argument("--exposure", type=int, default=None,
                        help="manual exposure_time_absolute (78..10000)")
    parser.add_argument("--brightness", type=int, default=None,
                        help="camera brightness, if supported")
    parser.add_argument("--focus", type=float, default=None,
                        help="manual focus value; disables autofocus")
    parser.add_argument("--v4l2-device", default=None,
                        help="device path, e.g. /dev/video2, for exact controls")
    parser.add_argument("--warmup-frames", type=int, default=60)
    args = parser.parse_args()
    for label in "ABCD":
        (args.output / label).mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"cannot open camera {args.camera}")
    configure_camera(cap, exposure=args.exposure, brightness=args.brightness,
                     focus=args.focus, v4l2_device=args.v4l2_device)
    if args.exposure is not None:
        print(f"requested manual exposure: {args.exposure}")
    if args.focus is not None:
        print(f"requested manual focus: {args.focus}")
    for _ in range(max(0, args.warmup_frames)):
        cap.read()
    print("Press A/B/C/D to save the detected white block; Q or Esc exits.")
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("camera frame read failed")
            break
        preview = frame.copy()
        rect, _ = largest_white_block(frame, args.saturation_max, args.value_min)
        if rect is not None:
            x, y, width, height = rect
            cv2.rectangle(preview, (x, y), (x + width, y + height), (255, 0, 0), 2)
            cv2.putText(preview, "A/B/C/D: save", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(preview, "No white block", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow("dataset collector", preview)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        label = chr(key).upper() if key else ""
        if label not in "ABCD" or rect is None:
            continue
        x, y, width, height = rect
        crop = frame[y : y + height, x : x + width]
        existing = sorted((args.output / label).glob("*.png"))
        path = args.output / label / f"{len(existing) + 1:04d}.png"
        if cv2.imwrite(str(path), crop):
            print(f"saved {label}: {path}")
    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
