#!/usr/bin/env python3
"""Train the OpenCV HOG+SVM model used by myvideo/run4."""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


SIZE = 64
HOG = cv2.HOGDescriptor((64, 64), (16, 16), (8, 8), (8, 8), 9)
CLASSES = ("A", "B", "C", "D")


def normalize(path: Path, black_threshold=180):
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        return None
    _, mask = cv2.threshold(image, black_threshold, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= 40]
    if not contours:
        return None
    x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
    crop = mask[y : y + h, x : x + w]
    side = max(w, h) + 8
    canvas = np.zeros((side, side), dtype=np.uint8)
    ox, oy = (side - w) // 2, (side - h) // 2
    canvas[oy : oy + h, ox : ox + w] = crop
    return cv2.resize(canvas, (SIZE, SIZE), interpolation=cv2.INTER_AREA)


def features(image):
    return np.asarray(HOG.compute(image, (8, 8), (0, 0)), dtype=np.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--validation-ratio", type=float, default=0.2)
    parser.add_argument("--black-threshold", type=int, default=180)
    args = parser.parse_args()
    if not args.dataset.is_dir():
        print(f"dataset directory not found: {args.dataset}", file=sys.stderr)
        return 2

    rng = np.random.default_rng(12345)
    class_samples = []
    for label, name in enumerate(CLASSES):
        files = sorted(p for p in (args.dataset / name).glob("*") if p.is_file())
        samples = [(features(image), label) for p in files
                   if (image := normalize(p, args.black_threshold)) is not None]
        if len(samples) < 2:
            print(f"class {name} needs at least 2 valid images", file=sys.stderr)
            return 2
        class_samples.append(samples)

    samples_per_class = min(len(samples) for samples in class_samples)
    train_x, train_y, valid_x, valid_y = [], [], [], []
    for samples in class_samples:
        if len(samples) > samples_per_class:
            samples = [samples[index] for index in
                       rng.permutation(len(samples))[:samples_per_class]]
        order = rng.permutation(len(samples))
        valid_count = max(1, int(round(len(samples) * args.validation_ratio)))
        for index in order[valid_count:]:
            train_x.append(samples[index][0])
            train_y.append(samples[index][1])
        for index in order[:valid_count]:
            valid_x.append(samples[index][0])
            valid_y.append(samples[index][1])

    train_x = np.vstack(train_x)
    train_y = np.asarray(train_y, dtype=np.int32).reshape(-1, 1)
    valid_x = np.vstack(valid_x)
    valid_y = np.asarray(valid_y, dtype=np.int32)
    svm = cv2.ml.SVM_create()
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setKernel(cv2.ml.SVM_RBF)
    svm.setC(10.0)
    svm.setGamma(0.01)
    svm.train(train_x, cv2.ml.ROW_SAMPLE, train_y)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    svm.save(str(args.output))

    predicted = svm.predict(valid_x)[1].astype(np.int32).reshape(-1)
    confusion = np.zeros((4, 4), dtype=np.int32)
    for truth, guess in zip(valid_y, predicted):
        confusion[truth, guess] += 1
    print(f"saved model: {args.output}")
    print(f"validation accuracy: {np.mean(predicted == valid_y):.3f}")
    print("per-class accuracy:")
    for index, name in enumerate(CLASSES):
        total = confusion[index].sum()
        print(f"  {name}: {(confusion[index, index] / total if total else 0.0):.3f}")
    print("confusion matrix (rows=true, cols=predicted):")
    print(confusion)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
