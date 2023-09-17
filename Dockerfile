FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

# Create non root user for pip
ENV USER ardupilot
RUN useradd -U -m ${USER} && \
    usermod -G users ${USER}

ENV USER_HOME /home/${USER}
ENV PATH "${USER_HOME}/.local/bin:${PATH}"

WORKDIR ${USER_HOME}

RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release sudo bash-completion \
    software-properties-common wget \
    rapidjson-dev gnupg \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
    gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 gstreamer1.0-pulseaudio

COPY Tools/environment_install/install-prereqs-ubuntu.sh ${USER_HOME}/ardupilot/Tools/environment_install/
COPY Tools/completion ${USER_HOME}/ardupilot/Tools/completion/

RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ardupilot
RUN chmod 0440 /etc/sudoers.d/ardupilot

USER ${USER}

ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN ${USER_HOME}/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y

# add waf alias to ardupilot waf to .bashrc
RUN echo "alias waf=\"\$HOME/ardupilot/waf\"" >> ~/.bashrc

# install Gazebo
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update && sudo apt-get install -y gz-garden

# install ArduPilot Gazebo plugin
#RUN mkdir ${USER_HOME}/ardupilot_gazebo_plugin && \
#    cd ${USER_HOME}/ardupilot_gazebo_plugin && \
#    git clone https://github.com/ArduPilot/ardupilot_gazebo && \
#    cd ardupilot_gazebo && \
#    mkdir build && cd build && \
#    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
#    make -j4

# just add directory to external mount ArduPilot Gazebo plugin
RUN mkdir ${USER_HOME}/ardupilot_gazebo_plugin

ENV GZ_SIM_SYSTEM_PLUGIN_PATH="${USER_HOME}/ardupilot_gazebo_plugin/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
ENV GZ_SIM_RESOURCE_PATH="${USER_HOME}/ardupilot_gazebo_plugin/ardupilot_gazebo/models:${USER_HOME}/ardupilot_gazebo_plugin/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"

# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

# Cleanup
RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV CCACHE_MAXSIZE=1G
