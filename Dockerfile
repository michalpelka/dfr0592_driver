# docker build -t dfr -f Dockerfile .
# docker run -it --privileged --net=host dfr
# in dcontainer
# ros2 run dfr0592_driver driver

FROM ros:humble

# Install necessary packages
RUN apt-get update && apt-get install -y \
    tmux \
    libi2c-dev \
    && rm -rf /var/lib/apt/lists/

# Install ROS 2 Humble packages
RUN apt-get update && apt-get install -y \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/


RUN mkdir -p /colcon && cd /colcon && git clone https://github.com/michalpelka/dfr0592_driver.git && . /opt/ros/humble/setup.sh && colcon build

# Source ROS 2 setup files
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /colcon/install/setup.bash" >> ~/.bashrc


# Set up entry point
CMD ["/bin/bash"]
