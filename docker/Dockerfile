# has to be called from the root dir of the repo with "sudo docker build -f docker/ros2/Dockerfile ." otherwise COPY will
# not work

ARG FROM_IMAGE=ros:jazzy
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM $FROM_IMAGE as builder
ARG OVERLAY_WS=/opt/ros/overlay_ws
WORKDIR $OVERLAY_WS/src
SHELL ["/bin/bash", "-c"]

RUN apt update && apt install python3.12-venv ros-jazzy-xacro python3-vcstool git ros-dev-tools default-jre graphviz graphviz-dev -y && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN vcs import --input https://raw.githubusercontent.com/cram2/semantic_world/main/rosinstall/semantic_world-https.rosinstall
RUN source /opt/ros/jazzy/setup.bash && cd $OVERLAY_WS && colcon build --symlink-install
RUN echo "source $OVERLAY_WS/install/setup.bash" >> ~/.bashrc

RUN python3 -m venv semantic_world-venv --system-site-packages  && source semantic_world-venv/bin/activate && pip install -U pip && pip install -U setuptools && pip install -r $OVERLAY_WS/src/semantic_world/requirements.txt

# Install the Multiverse Parser outside of the overlay workspace
WORKDIR /opt/ros
RUN git clone https://github.com/Multiverse-Framework/Multiverse-Parser.git
RUN source $OVERLAY_WS/src/semantic_world-venv/bin/activate && pip install -r Multiverse-Parser/requirements.txt && pip install pyside6 pyopengl jinja2 &&  ./Multiverse-Parser/setup.sh --usd && pip install -e Multiverse-Parser

RUN source /opt/ros/jazzy/setup.bash && cd $OVERLAY_WS && colcon build --symlink-install
RUN echo "source $OVERLAY_WS/install/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /
ENTRYPOINT ["bash", "/entrypoint.sh"]
