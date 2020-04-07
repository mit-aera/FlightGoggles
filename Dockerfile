FROM osrf/ros:kinetic-desktop-full

# Use bash for commands
SHELL ["/bin/bash", "-c"]

# Delete old ROS apt distribution key (revoked)
# https://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get clean

# Install ROS
RUN apt update \
    && apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools \
    && rosdep update \
    && apt-get clean \
    && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# Install FlightGoggles dependencies
RUN mkdir -p /root/catkin_ws/src \
    && cd /root/catkin_ws/ \
    && catkin init \
    && echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc \
    && cd /root/catkin_ws/src/ \
    && wstool init \
    && wstool merge https://raw.githubusercontent.com/mit-fast/FlightGoggles/master/flightgoggles.rosinstall \
    && apt-get update \
    && wstool update \
    && cd /root/catkin_ws/ \
    && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y \
    && apt install -y libzmqpp-dev libeigen3-dev \
    && apt-get clean


# Copy FG into the Docker Image
COPY ./ /root/catkin_ws/src/flightgoggles/

# Build FG ROS bindings
RUN source /opt/ros/kinetic/setup.bash \
    && cd /root/catkin_ws/ \
    && catkin build \ 
        -DFLIGHTGOGGLES_DOWNLOAD_BINARY=OFF \
        -DCMAKE_BUILD_TYPE=Release

# Allow for incoming ports from FG
EXPOSE 10253/tcp
EXPOSE 10254/tcp