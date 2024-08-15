FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-pip \
    wget \
    unzip \
    apt-utils \
    git \
    && rm -rf /var/lib/apt/lists/*

# Run the installation script from pixi.sh
RUN curl -fsSL https://pixi.sh/install.sh | bash

# system configuration
RUN chmod o+w /etc \
    && chmod o+w /etc/group \
    && chmod o+w /etc/passwd \
    && chmod o+rw /etc/shadow \
    && echo "Defaults !env_reset" >> /etc/sudoers \
    && echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# create and configure local user
RUN groupadd -g 1000 user \
    && useradd -u 1000 -g user -G sudo -s /bin/bash -d /tmp/user -m -l user

RUN chown -R 1000:1000 /tmp/user \
    && echo "alias ..='cd ..'" >> /tmp/user/.bashrc \
    && echo "export PATH=/tmp/user/.local/bin:$PATH" >> /tmp/user/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /tmp/user/.bashrc

# Initialize a Pixi project
RUN pixi init jupyter_ros_ws -c robostack-staging -c conda-forge

# Change directory to the Pixi project
WORKDIR /jupyter_ros_ws

# Add jupyter-ros and ROS2 dependencies
RUN pixi add jupyter-ros ros-humble-desktop 

# Enable the jupyter-ros server extension
RUN pixi run jupyter serverextension enable jupyros

# Expose the Jupyter notebook port
EXPOSE 8888

# Configure ROS workspace
# WORKDIR /tmp/user/jupyter_ros_ws

RUN . /opt/ros/humble/setup.sh && colcon build

# configure startup
COPY ./docker/usr/local/bin /usr/local/bin
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Define the entrypoint and default command
# ENTRYPOINT ["pixi"]
CMD ["run", "jupyter", "notebook", "--allow-root", "--ip=0.0.0.0", "--no-browser"]