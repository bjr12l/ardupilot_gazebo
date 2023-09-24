FROM ardupilot:latest

USER root

RUN apt update && apt install -y \
    software-properties-common wget \
    rapidjson-dev gnupg cmake \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
    gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 gstreamer1.0-pulseaudio

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