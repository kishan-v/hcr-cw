# Use NVIDIA L4T base image for Jetson devices
FROM nvcr.io/nvidia/l4t-base:r35.3.1

# Install build tools, dependencies for OpenCV & GStreamer, and libusb for libuvc
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    software-properties-common \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libdc1394-22-dev \
    pkg-config \
    libusb-1.0-0-dev \
 && rm -rf /var/lib/apt/lists/*

# Install Python 3.12 from deadsnakes PPA
RUN add-apt-repository ppa:deadsnakes/ppa -y && \
    apt-get update && apt-get install -y \
    python3.12 \
    python3.12-dev \
    python3.12-venv \
    python3.12-distutils \
 && rm -rf /var/lib/apt/lists/*

# Install pip for Python 3.12
RUN wget https://bootstrap.pypa.io/get-pip.py && \
    python3.12 get-pip.py && rm get-pip.py

# Update alternatives so that "python" points to Python 3.12
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.12 1

# Clone and build the gstthetauvc plugin
RUN git clone https://github.com/nickel110/gstthetauvc.git /opt/gstthetauvc
WORKDIR /opt/gstthetauvc/thetauvc
RUN make -j$(($(nproc) - 1))
# Copy gstthetauvc.so into the gstreamer plugin directory 
RUN cp /opt/gstthetauvc/thetauvc/gstthetauvc.so /usr/lib/aarch64-linux-gnu/gstreamer-1.0/

# Clone OpenCV and opencv_contrib repositories
RUN git clone https://github.com/opencv/opencv.git /opt/opencv
RUN git clone https://github.com/opencv/opencv_contrib.git /opt/opencv_contrib

# Build OpenCV from source with GStreamer support and Python bindings
WORKDIR /opt/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
          -D WITH_GSTREAMER=ON \
          -D PYTHON_EXECUTABLE=$(which python3.12) \
          -D BUILD_opencv_python3=ON \
          -D PYTHON3_PACKAGES_PATH="$(python3.12 -c 'from distutils.sysconfig import get_python_lib; print(get_python_lib())')" \
          ..
RUN make -j$(($(nproc) - 1))
RUN make install

# Build the Python wheel for the opencv python_loader
WORKDIR /opt/opencv/build/python_loader
RUN python3.12 setup.py bdist_wheel

# Create the directory for the project files
RUN mkdir -p /opt/hcr-cw

# Install the python project requirements
COPY ../../requirements.txt /opt/hcr-cw/
RUN python3.12 -m pip install /opt/hcr-cw/requirements.txt
# Install the Python wheel for openCV (built above)
RUN python3.12 -m pip install /opt/opencv/build/python_loader/dist/*.whl

COPY entrypoint.sh /opt/hcr-cw/
RUN chmod +x /opt/hcr-cw/entrypoint.sh

COPY ../webrtc_transmitter.py \
../websocket_signaling.py \
../computer_vision.py \
/opt/hcr-cw/

# CMD ["./entrypoint.sh"]
CMD ["python3.12", "/opt/hcr-cw/webrtc_transmitter.py"]
