FROM osrf/ros:kinetic-desktop-full

# Install ROS
RUN apt update && apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools
RUN rosdep update
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# Install FlightGoggles
RUN mkdir -p /root/catkin_ws/src && cd /root/catkin_ws/ && catkin init
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN cd /root/catkin_ws/src/ && wstool init && wstool merge https://raw.githubusercontent.com/mit-fast/FlightGoggles/master/flightgoggles.rosinstall && wstool update
RUN cd /root/catkin_ws/ && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
RUN apt update && apt install -y libzmqpp-dev libeigen3-dev
# Do not run catkin build. 

# Allow for incoming ports from FG
EXPOSE 10253/tcp
EXPOSE 10254/tcp