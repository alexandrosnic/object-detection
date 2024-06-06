FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-pip \
    wget \
    unzip \
    apt-utils \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install torch torchvision

# Clone YOLOv5 and install its requirements
RUN git clone https://github.com/ultralytics/yolov5 /tmp/yolov5 \
    && pip3 install -r /tmp/yolov5/requirements.txt

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

# Configure ROS workspace
WORKDIR /tmp/user/object_detection_ws


# COPY . /tmp/user/object_detection_ws/src/yolo_detector



RUN . /opt/ros/humble/setup.sh && colcon build

# configure startup
COPY ./docker/usr/local/bin /usr/local/bin
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
