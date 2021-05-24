import subprocess
from evds_to_pcds import evds_to_pcds
from get_distances import get_distances
from rgb_to_laser_angle import transform_colors_to_laser_angles

d = "/home/brano/Projects/thesis/virtual_error_measurements"
pcd_glob = "/home/brano/Projects/thesis/virtual_error_measurements/*.pcd"
txt_glob = "/home/brano/Projects/thesis/virtual_error_measurements/*.txt"
blender = "/home/brano/Projects/blensor/cmake-build-debug/bin/blender"
error_script = "/home/brano/Projects/thesis/scan_for_error.py"


# 1. run the virtual scans
subprocess.run([blender, "-b", "-P",  error_script])

# 2. convert .evd files to .pcd files
evds_to_pcds(d)

# 3. get the error to plane distances
get_distances(pcd_glob)

# 4. convert the RGB value of the laser to the laser id
transform_colors_to_laser_angles(txt_glob)

# 5. make violin plots, stddevs or some other metric comparing to the real data