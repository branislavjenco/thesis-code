# Virtual LiDAR error models in point cloud compression
Repository for my Master thesis "Virtual LiDAR error models in point cloud compression".

## Contents
The most important parts of the repository are listed below.

### `noise_distributions.ipynb`

This notebook accumulates the analysis of the errors from the real and virtual scans.

### `pccarena_results.ipynb`

This notebook accumulates the analysis of the results from the PCC Arena benchmarks. The input to this analysis is the `summary.csv` file which contains the measured results.

### `blend.empty`

This file is the basis empty scene file used for all the scanning procedures in BlenSor.

### `scan_for_error.py`

This script performs the virtual error measurements for a given error model with a floor and wall placed in the scene.

### `error_pipeline.py`

Wraps the `scan_for_error.py` and adds some filesystem scaffolding.

### `s3dis_can.py`

This script performs BlenSor scans in a given mesh file at given locations.

### `s3dis_pipeline.py`

Wraps the `s3dis_scan.py` and adds some filesystem scaffolding as well as the cropping by bounding box operation implemented in `crop_by_bounding_box.py`.

### `exploration_scan_script.py`

This script was used for the construction of the scanning pipeline and other explorations.

### `evd_reader.py`

Contains classes to read BlenSor's `.evd` file and output a `.txt` or `.pcd` file.

## Other work

Apart from the work in this repository, modifications to Indoor reconstruction++ were made as a Pull Request to the original repository, available [here](https://github.com/henryhansen23/Indoor-reconstruction-plus-plus/pull/4). The work done on BlenSor is available in a fork of the original repository found [here](https://github.uio.no/branisj/blensor). 
