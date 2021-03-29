import struct
import traceback

class EvdReader:
    rayIndex = 0
    raysInScan = 0
    fileHandle = None

    def __init__(self, filename):
        self.openEvdFile(filename)

    def openEvdFile(self, filename):
        self.fileHandle = open(filename, "rb")

    def getNextRay(self):
        if self.rayIndex >= self.raysInScan:
            data = struct.unpack("i", self.fileHandle.read(4))
            self.raysInScan = data[0]
            if data[0] == -1:
                return data[0]
            self.rayIndex = 0
        ray = struct.unpack("14dQ", self.fileHandle.read(15 * 8))
        self.rayIndex += 1
        return ray

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

    def writePCLFile(self, iterator, width):
        width = width
        height = 1

        try:
            pcl_noisy = open("%s_noisy.pcd" % self.filename, "w")
            if self.output_labels:
                pcl_noisy.write(PCL_HEADER_WITH_LABELS % (width, height, width * height))
            else:
                pcl_noisy.write(PCL_HEADER % (width, height, width * height))
            idx = 0
            for e in iterator:
                idx += 1
                self.write_point(pcl_noisy, e, self.output_labels)
            pcl_noisy.close()
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


def it(reader):
    count = 0
    while True:
        ray = reader.getNextRay()
        if ray == -1:
            break
        else:
            yield ray
        count += 1
    print(f"Finished: {count - 1}")


er = EvdReader("test_0.evd")
writer = PCLWriter("test_0.pcl")
writer.writePCLFile(it(er), 42)

