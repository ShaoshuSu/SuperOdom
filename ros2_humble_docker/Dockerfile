# Author: TaeYoung Kim
# email: tyoung96@yonsei.ac.kr

FROM --platform=linux/arm64 ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    software-properties-common \
    gnupg \
    lsb-release \
    curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y ros-humble-desktop-full \
    python3-colcon-common-extensions \
    python3-colcon-core \
    python3-colcon-ros \
    python3-colcon-test-result \
    && apt-get clean

# Install PCL & Eigen & essential libraries
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        cmake \
        cppcheck \
        gdb \
        git \
        sudo \
        vim \
        wget \
        tmux \
        curl \
        less \
        htop \
        libsm6 libxext6 libgl1-mesa-glx libxrender-dev \
 && apt-get clean

RUN sudo apt-get update \
 && sudo -E apt-get install -y \
    tzdata \
 && sudo ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime \
 && sudo dpkg-reconfigure --frontend noninteractive tzdata \
 && sudo apt-get clean
 
RUN apt-get update && apt-get install -y cmake libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget unzip git python3-pip
RUN apt-get install -y ros-humble-tf2 ros-humble-cv-bridge ros-humble-pcl-conversions ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-pcl-ros

RUN apt-get install -y libceres-dev 
    

RUN apt-get install -y python-tk python3-pip

# Python 3.
RUN pip3 install --upgrade pip
RUN pip3 install --no-cache-dir numpy scipy matplotlib ipython opencv-python 
RUN pip3 install --no-cache-dir pandas visdom scikit-image scikit-learn numba Pillow
RUN pip3 install --no-cache-dir pyyaml pycryptodome bagpy Cython

RUN apt-get -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN rosdep init
RUN rosdep update
RUN pip3 install -U catkin_tools gym
RUN apt-get -y install ros-humble-grid-map-msgs ros-humble-grid-map

# Install livox SDK
WORKDIR /root/
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /root/Livox-SDK2
RUN mkdir build
WORKDIR /root/Livox-SDK2/build
RUN cmake .. && make -j2 && make install

WORKDIR /root/
RUN git clone http://github.com/strasdat/Sophus.git \
    && cd Sophus && git checkout 97e7161 \
    && mkdir build && cd build && cmake .. -DBUILD_TESTS=OFF \
    && make -j8 && sudo make install
WORKDIR /root/
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && git checkout 4abef92 &&\
    mkdir build && \
    cd build && \
    cmake  \
          -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
           .. && \
    make -j6 && \
    sudo make install


RUN /bin/bash -c '. /opt/ros/humble/setup.bash;'

RUN pip install rerun-sdk
RUN pip install numpy==1.24.3 matplotlib==3.7.1
RUN pip install --upgrade packaging

# Install Intel RealSense SDK
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
    apt-get update && \
    apt-get install -y librealsense2-utils librealsense2-dev && \
    apt-get clean

RUN apt-get install -y ros-humble-realsense2-*


WORKDIR /root/ros2_ws

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]

























