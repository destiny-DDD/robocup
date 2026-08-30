# myvideo HOG+SVM Recognition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a pure OpenCV HOG+SVM `run4` mode that detects black A/B/C/D letters on white blocks over a green background and overlays a stable label.

**Architecture:** Reuse one preprocessing contract for training and inference: normalize a character crop to a binary `64x64` image, extract fixed HOG features, and classify with a four-class OpenCV SVM. Runtime first segments white blocks in HSV, extracts a black-letter contour from each block, ranks candidates deterministically, and applies a three-frame confirmation filter.

**Tech Stack:** ROS 2 C++, OpenCV `imgproc`, `objdetect`, and `ml`; Python training utility using `cv2` only.

**Spec:** `docs/superpowers/specs/2026-08-31-myvideo-hog-svm-design.md`

## Global Constraints

- Runtime and training use OpenCV only; no PyTorch, ONNX, or deep-learning runtime.
- Classes are exactly A/B/C/D with labels 0/1/2/3.
- Input normalization is binary single-channel `64x64` with fixed HOG parameters.
- Missing models and missing detections must not terminate the ROS node.

### Task 1: Add shared runtime model state and preprocessing helpers

**Files:**
- Modify: `src/myvideo/include/myvideo/myvideo.hpp`
- Modify: `src/myvideo/src/myvideo.cpp`

- [x] Declare `run4`, HOG/SVM members, model path and HSV/threshold parameters, plus private helpers for model loading, character normalization, and feature extraction.
- [x] Initialize parameters in the constructor, resolve the default model path through `ament_index_cpp`, and load the SVM once with an error log on failure.
- [x] Implement helpers so a binary character mask is cropped by its largest valid contour, padded to a square, resized to `64x64`, and converted to one-row HOG features using `HOGDescriptor(Size(64,64), Size(16,16), Size(8,8), Size(8,8), 9)`.

### Task 2: Implement `run4` white-block and black-letter pipeline

**Files:**
- Modify: `src/myvideo/src/myvideo.cpp`

- [x] Capture a frame, segment white blocks with configurable HSV low-saturation/high-value thresholds, close morphology, and contour geometry filters.
- [x] Rank white-block candidates by area, then extract a black-letter mask inside each ROI using grayscale thresholding and morphology.
- [x] Normalize the selected character, predict with SVM, draw the selected block/letter boxes, and apply three consecutive matching predictions before changing the confirmed label.
- [x] Render `LETTER: A`..`D`, `LETTER: ?`, or `SVM unavailable` at the top-left and return `false` only when frame capture fails.

### Task 3: Wire serial mode 4 and package assets

**Files:**
- Modify: `src/myvideo/src/myvideo.cpp`
- Modify: `src/myvideo/CMakeLists.txt`
- Modify: `src/myvideo/package.xml`
- Create: `src/myvideo/model/.gitkeep`

- [x] Extend serial validation to accept 1..4 and dispatch `case 4` to `run4` while preserving modes 1..3.
- [x] Install the model directory and the training utility from the package share tree; keep OpenCV as the only additional library requirement.

### Task 4: Add pure OpenCV training utility and usage documentation

**Files:**
- Create: `src/myvideo/training/train_hog_svm.py`
- Create: `src/myvideo/training/README.md`

- [x] Load `dataset/A`..`dataset/D`, apply the same binary crop/pad/resize and HOG parameters, train a C_SVC RBF SVM, and save `abcd_hog_svm.yml`.
- [x] Split each class deterministically into training and validation subsets and print overall/per-class accuracy plus a confusion matrix.
- [x] Document collection expectations, command-line usage, and the required runtime model location.

### Task 5: Verify build and static behavior

- [x] Run `git diff --check`.
- [x] Build `myvideo` with the workspace OpenCV/ROS environment.
- [x] Run the training utility help and a missing-dataset error path; confirm they fail with actionable messages.
- [x] Inspect the final diff for mode 1..3 regressions and model-path/install consistency.
