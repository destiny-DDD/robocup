# HOG+SVM training

This utility uses only OpenCV (`cv2`) and produces the model consumed by `myvideo` mode 4.

Collect cropped white-object images with a black A/B/C/D letter into:

```text
dataset/A/
dataset/B/
dataset/C/
dataset/D/
```

You can collect crops automatically with the camera (the default camera index matches `myvideo`):

```bash
python3 training/collect_dataset.py training/dataset --camera 2
```

The preview marks the largest detected white block. Put one letter on that block and press `A`, `B`, `C`, or `D` to save a crop in the corresponding directory. Press `Q` or `Esc` to quit. Adjust `--saturation-max` and `--value-min` if the white block is not detected.

To generate black-letter-only samples without changing an existing dataset, run:

```bash
python3 training/clean_dataset.py training/dataset
```

It keeps `training/dataset/` unchanged and writes white-background, black-letter `64x64` images to `training/dataset_clean/`. Green-background contours touching an image edge are ignored. Images with no reliable internal letter are listed in `training/dataset_clean/clean_dataset_report.txt` and are not used for training.

The collector discards 60 startup frames so autofocus/exposure can settle. If the camera keeps hunting focus, set a manual focus value supported by that camera:

```bash
python3 training/collect_dataset.py training/dataset --camera 2 --focus 20
```

Try values around the camera's supported range until the A4 print is sharp at the intended distance. The `--focus` option disables autofocus before applying the value.

For this camera, the collector can set V4L2 exposure directly after opening the stream:

```bash
python3 training/collect_dataset.py training/dataset --camera 2 \
  --v4l2-device /dev/video2 --exposure 78
```

Use `--exposure 100`, `150`, or `200` if the minimum is too dark. `--brightness -20` is also available. The V4L2 settings are applied after OpenCV opens the camera, so opening the collector will not immediately overwrite them.

Use at least a few dozen images per class, varying distance, rotation, lighting and object size. Train from the package directory with:

```bash
python3 training/train_hog_svm.py training/dataset model/abcd_hog_svm.yml
```

The default black-letter threshold is `180`, matching the runtime node. You can override it with `--black-threshold`, but keep the same value in the ROS `black_threshold` parameter when running `run4`.

Copy or install the resulting file as `share/myvideo/model/abcd_hog_svm.yml`. Runtime preprocessing uses the same binary crop and `64x64` HOG contract as this script.
