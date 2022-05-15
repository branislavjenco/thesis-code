import subprocess
import os
import glob
from evds_to_pcds import evds_to_pcds
from get_distances import get_distances
from rgb_to_laser_angle import transform_colors_to_laser_angles

# Wrapper around the scan_for_error script which performs the full virtual error pipeline

dist_folder = "virtual_error_measurements"
blender = "<blensor executable>"
error_script = "./scan_for_error.py"

errors = ["none", "default", "base_vlp16", "divergence", "incidence"]

def clear_glob(g):
    files = glob.glob(g)
    for f in files:
        os.remove(f)


for error in errors:
    errors_folder = f"./{dist_folder}"

    # Make directory if doesn't exist
    os.makedirs(errors_folder, exist_ok=True)
    os.makedirs(f"{errors_folder}/{error}", exist_ok=True)

    pcd_glob = f"{errors_folder}/{error}/*.pcd"
    evd_glob = f"{errors_folder}/{error}/*.evd"
    txt_glob = f"{errors_folder}/{error}/*.txt"
    info_glob = f"{errors_folder}/{error}/*.info"

    # Clear the files
    clear_glob(pcd_glob)
    clear_glob(evd_glob)
    clear_glob(txt_glob)
    clear_glob(info_glob)

    # Run the virtual scans
    subprocess.run([blender, "-b", "-P",  error_script, "--", "-e", error])

    # Convert .evd files to .pcd files
    evds_to_pcds(evd_glob)

    # Get the error to plane distances
    get_distances(pcd_glob)

    # Convert the RGB value of the laser to the laser angle
    transform_colors_to_laser_angles(txt_glob)
