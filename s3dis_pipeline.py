import subprocess
from evds_to_pcds import evds_to_pcds
from crop_by_bounding_box import main as crop

evd_glob = "/home/branislav/repos/thesis/s3dis_scans_2/*.evd"
loc_glob = "/home/branislav/repos/thesis/locations_pruned*.json"
blender = "/home/branislav/repos/blensor2/cmake-build-release/bin/blender"
scan_script = "/home/branislav/repos/thesis/s3dis_scan.py"
full_s3dis_dir = "/home/branislav/repos/s3dis/"


# 1. run the virtual scans
# subprocess.run([blender, "-b", "-P",  scan_script, "--", "-l", loc_glob, "-s", full_s3dis_dir])

# 2. convert .evd files to .pcd files
# evds_to_pcds(evd_glob)

# 3. crop by BB
crop()




