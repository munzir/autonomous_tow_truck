FROM munzirzafar/towtruck:hardware_compatible

# Copy alias script into container
COPY ros2_aliases.sh /root/ros2_aliases.sh

# Source ROS 2 and workspace setup in every shell (before aliases)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/autonomous_tow_truck/install/setup.bash" >> /root/.bashrc && \
    echo "source /root/autonomous_tow_truck/install/local_setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_aliases.sh" >> /root/.bashrc

# Set working directory
WORKDIR /root/autonomous_tow_truck

# Default to interactive shell
CMD ["bash"]

