FROM ardupilot:latest

USER root

RUN apt update && apt install -y \
    software-properties-common wget \
    rapidjson-dev gnupg build-essential cmake \
    pkg-config libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
    gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 gstreamer1.0-pulseaudio


# Install OpenCV
RUN git clone https://github.com/opencv/opencv.git &&  \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D BUILD_PYTHON3=ON \
          -D BUILD_OPENCV_PYTHON2=OFF \
          -D ENABLE_CXX11=ON \
          -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
          -D BUILD_TIFF=ON \
          -D WITH_CUDA=OFF \
          -D ENABLE_AVX=OFF \
          -D WITH_OPENGL=OFF \
          -D WITH_OPENCL=OFF \
          -D WITH_IPP=OFF \
          -D WITH_TBB=ON \
          -D BUILD_TBB=ON \
          -D WITH_V4L=OFF \
          -D WITH_VTK=OFF \
          -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          -D BUILD_SHARED_LIBS=ON \
          -D OPENCV_GENERATE_PKGCONFIG=ON \
          -D WITH_GSTREAMER=ON \
          .. && \
    make -j4 && \
    make install && \
    ldconfig

# install Gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y gz-garden

COPY  --chown=${USER} . ${USER_HOME}/ardupilot_gazebo_plugin

USER ${USER}

# install ArduPilot Gazebo plugin
RUN cd ${USER_HOME}/ardupilot_gazebo_plugin && \
   mkdir build && cd build && \
   cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
   make -j4

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=${USER_HOME}/ardupilot_gazebo_plugin/build
ENV GZ_SIM_RESOURCE_PATH=${USER_HOME}/ardupilot_gazebo_plugin/models:${USER_HOME}/ardupilot_gazebo_plugin/worlds

CMD gz sim -v4 -r --render-engine=ogre iris_runway.sdf