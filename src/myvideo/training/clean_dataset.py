#!/usr/bin/env python3
"""Extract black letters from raw white-paper images without changing originals."""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def extract_letter(image, black_threshold=180):
    """Return a white-background, black-letter 64x64 sample, or None."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    dark = gray < black_threshold
    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    # Black/gray print has low saturation; this camera renders dark print as
    # blue-gray with Hue above 105. The green background remains below 105.
    not_green = (saturation < 40) | (hue < 35) | (hue > 105)
    mask = np.where(dark & not_green, 255, 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    height, width = mask.shape
    candidates = []
    for component in range(1, count):
        x, y, box_width, box_height, area = stats[component]
        touches_edge = x <= 1 or y <= 1 or x + box_width >= width - 1 or y + box_height >= height - 1
        if area >= 40 and not touches_edge:
            candidates.append((area, component, (x, y, box_width, box_height)))
    if not candidates:
        return None
    _, component, (x, y, box_width, box_height) = max(
        candidates, key=lambda item: item[0]
    )
    letter = np.where(labels[y : y + box_height, x : x + box_width] == component,
                      255, 0).astype(np.uint8)
    side = max(box_width, box_height) + 8
    canvas = np.zeros((side, side), dtype=np.uint8)
    offset_x = (side - box_width) // 2
    offset_y = (side - box_height) // 2
    canvas[offset_y : offset_y + box_height, offset_x : offset_x + box_width] = letter
    normalized = cv2.resize(canvas, (64, 64), interpolation=cv2.INTER_AREA)
    return cv2.bitwise_not(normalized)


def process_dataset(dataset, output, black_threshold):
    labels = "ABCD"
    files = [path for label in labels for path in sorted((dataset / label).glob("*.png"))]
    if not files:
        raise ValueError(f"no PNG images found in {dataset}")
    if output.exists():
        raise ValueError(f"output directory already exists: {output}")
    for label in labels:
        (output / label).mkdir(parents=True)

    failed = []
    for path in files:
        image = cv2.imread(str(path))
        cleaned = extract_letter(image, black_threshold) if image is not None else None
        if cleaned is None:
            failed.append(str(path.relative_to(dataset)))
            continue
        destination = output / path.relative_to(dataset)
        if not cv2.imwrite(str(destination), cleaned):
            failed.append(str(path.relative_to(dataset)))
    report = output / "clean_dataset_report.txt"
    report.write_text("\n".join(failed) + ("\n" if failed else ""), encoding="utf-8")
    return len(files), len(failed), report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--black-threshold", type=int, default=180)
    args = parser.parse_args()
    output = args.output or args.dataset.with_name(args.dataset.name + "_clean")
    try:
        total, failed, report = process_dataset(
            args.dataset, output, args.black_threshold
        )
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    print(f"wrote {total - failed}/{total} cleaned images to: {output}")
    print(f"unprocessed-image report: {report}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
