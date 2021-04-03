import struct
import traceback
import argparse
import os


total_count = 0

class EvdReader:
    rayIndex = 0
    raysInScan = 0
    fileHandle = None

    def __init__(self, filename):
        self.openEvdFile(filename)

    def openEvdFile(self, filename):
        self.fileHandle = open(filename, "rb")

    def getNextRay(self):
        global total_count
        if self.rayIndex >= self.raysInScan:
            data = struct.unpack("i", self.fileHandle.read(4))
            self.raysInScan = data[0]
            if data[0] == -1:
                return data[0]
            self.rayIndex = 0
        ray = struct.unpack("14dQ", self.fileHandle.read(15 * 8))
        if total_count < 5:
            print(ray)
        self.rayIndex += 1
        total_count += 1
        return ray

    def getRays(self):
        count = 0
        while True:
            ray = self.getNextRay()
            if ray == -1:
                break
            else:
                yield ray
            count += 1
        print(f"Finished: {count - 1}")

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

    def writePCLFile(self, iterator):
        height = 1

        try:
            with open("tmp.pcd", "w") as pcl_tmp:
                count = 0
                for e in iterator:
                    count += 1
                    self.writePoint(pcl_tmp, e, self.output_labels)
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

    def writePoint(self, pcl_noisy, e, output_labels):
        # Storing color values packed into a single floating point number???
        # That is really required by the pcl library!
        color_uint32 = (int(e[11]) << 16) | (int(e[12]) << 8) | (int(e[13]))
        values = struct.unpack("f", struct.pack("I", color_uint32))

        if output_labels:
            pcl_noisy.write("%f %f %f %.15e %d\n" % (float(e[8]), float(e[9]), float(e[10]), values[0], int(e[11])))
        else:
            pcl_noisy.write("%f %f %f %.15e\n" % (float(e[8]), float(e[9]), float(e[10]), values[0]))


parser = argparse.ArgumentParser()
parser.add_argument(
    '-f',
    help = 'EVD file to read'
)

args = parser.parse_args()
f = args.f

reader = EvdReader(f)
writer = PCLWriter(f"{os.path.splitext(f)[0]}")
writer.writePCLFile(reader.getRays())

