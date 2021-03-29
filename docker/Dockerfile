FROM continuumio/miniconda3

# conda part
RUN conda install -y -c conda-forge numpy && conda install -y -c conda-forge igl && conda install -y -c conda-forge trimesh

# PyMesh part
WORKDIR /root/
ARG BRANCH="main"
ARG NUM_CORES=2
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    git \
    cmake \
    libgmp-dev \
    libmpfr-dev \
    libgmpxx4ldbl \
    libboost-dev \
    libboost-thread-dev \
    zip unzip patchelf && \
    apt-get clean && \
    git clone --single-branch -b $BRANCH https://github.com/PyMesh/PyMesh.git

ENV PYMESH_PATH /root/PyMesh
ENV NUM_CORES $NUM_CORES
WORKDIR $PYMESH_PATH

RUN git submodule update --init && \
    pip install -r $PYMESH_PATH/python/requirements.txt && \
    ./setup.py bdist_wheel && \
    rm -rf build_3.7 third_party/build && \
    python $PYMESH_PATH/docker/patches/patch_wheel.py dist/pymesh2*.whl && \
    pip install dist/pymesh2*.whl && \
    python -c "import pymesh; pymesh.test()"

RUN conda install -y -c conda-forge networkx
RUN conda install -y -c conda-forge shapely
RUN conda install -y -c conda-forge pillow
RUN conda install -y -c conda-forge rtree

WORKDIR /
COPY pipeline.py /
ENTRYPOINT ["python", "pipeline.py"]
