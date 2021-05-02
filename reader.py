import struct
import traceback
import argparse
import os
import glob


# the EVD reader and PCL writer classes are adapted from the blensor repository

class EvdReader:
    ray_index = 0
    rays_in_scan = 0
    file_handle = None
    total_count = 0

    def __init__(self, filename):
        self.open_evd_file(filename)

    def open_evd_file(self, filename):
        self.file_handle = open(filename, "rb")

    def get_next_ray(self):
        if self.ray_index >= self.rays_in_scan:
            data = struct.unpack("i", self.file_handle.read(4))
            self.rays_in_scan = data[0]
            if data[0] == -1:
                # This means we got to the end of the file
                return data[0]
            if data[0] == 0:
                # This means that for some reason there are 0 scans in the next "block"
                # We should just ignore this and have a look at the next 4 bytes
                return self.get_next_ray()
            self.ray_index = 0
        # 14 doubles (8 bytes) and one long long integer (8 bytes)
        ray = struct.unpack("14dQ", self.file_handle.read(15 * 8))
        self.ray_index += 1
        self.total_count += 1
        return ray

    def get_rays(self):
        count = 0
        while True:
            ray = self.get_next_ray()
            if ray == -1:
                break
            else:
                yield ray
            count += 1
        print(f"Finished: {count - 1}")


class MultiEvdReader:
    total_count = 0
    def __init__(self, filenames):
        self.filenames = filenames

    def get_rays(self):
        for filename in self.filenames:
            reader = EvdReader(filename)
            for ray in reader.get_rays():
                yield ray
                self.total_count += 1




PCL_HEADER = """# .PCD v.7 - Exported by BlenSor
VERSION .7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH %d
HEIGHT %d
VIEWPOINT 0 0 0 1 0 0 0
POINTS %d
DATA ascii
"""

PCL_HEADER_WITH_LABELS = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z rgb label
SIZE 4 4 4 4 4
TYPE F F F F U
COUNT 1 1 1 1 1
WIDTH %d
HEIGHT %d
VIEWPOINT 0 0 0 1 0 0 0
POINTS %d
DATA ascii
"""

class PCLWriter:
    width = 0
    height = 0

    def __init__(self, filename, output_labels=True):
        self.filename = filename
        self.output_labels = output_labels

    def write_pcl_file(self, iterator):
        height = 1

        try:
            with open("tmp.pcd", "w") as pcl_tmp:
                count = 0
                for e in iterator:
                    count += 1
                    self.write_point(pcl_tmp, e, self.output_labels)
                print(f"count {count}")

            with open("tmp.pcd", "r") as pcl_tmp:
                with open("%s.pcd" % self.filename, "w") as pcl_final:
                    width = count - 2
                    if self.output_labels:
                        pcl_final.write(PCL_HEADER_WITH_LABELS % (width, height, width * height))
                    else:
                        pcl_final.write(PCL_HEADER % (width, height, width * height))
                    for line in pcl_tmp:
                        pcl_final.write(line)
            os.remove("tmp.pcd")

        except Exception as e:
            traceback.print_exc()

    def write_point(self, pcl_noisy, e, output_labels):
        # Storing color values packed into a single floating point number???
        # That is really required by the pcl library!
        color_uint32 = (int(e[11]) << 16) | (int(e[12]) << 8) | (int(e[13]))
        values = struct.unpack("f", struct.pack("I", color_uint32))

        if output_labels:
            pcl_noisy.write("%f %f %f %.15e %d\n" % (float(e[8]), float(e[9]), float(e[10]), values[0], int(e[11])))
        else:
            pcl_noisy.write("%f %f %f %.15e\n" % (float(e[8]), float(e[9]), float(e[10]), values[0]))


# parser = argparse.ArgumentParser()
# parser.add_argument(
#     '-f',
#     help = 'EVD file to read'
# )
# parser.add_argument(
#     '-g',
#     help = 'glob - use a wildcard to specify multiple evd files, these will be made into a single pcl file'
# )
#
# args = parser.parse_args()
# f = args.f
# g = args.g
#
# if g:
#     reader = MultiEvdReader(sorted(glob.glob(g)))
#     writer = PCLWriter(f"{os.path.splitext(g)[0]}")
#     writer.write_pcl_file(reader.get_rays())
# elif f:
#     reader = EvdReader(f)
#     writer = PCLWriter(f"{os.path.splitext(f)[0]}")
#     writer.write_pcl_file(reader.get_rays())
#
