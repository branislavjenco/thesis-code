# C++ scripts

This folder contains scripts written in C++ that were used during the writing of this thesis along with a `CMakeLists.txt` file.

## `get_plane_distances`

This script was used to crop the point clouds obtained from the real VLP-16 sensor and calculate the plane residual data for each point. 

## `set_laser_id_as_label`
Takes a point cloud saved in the `.pcd` format with labels that correspond to vertical laser channel angles and translates these to the corresponding laser ids, based on the VLP16 manual lookup table. Subsequently, the ID is scaled to the interval [0, 255] so that it's interpreted as color by most point cloud visualization tools.

## `get_centroids`

This script was used for exploration of the error characteristics of clusters of points seen in real VLP-16 point clouds.