#!/usr/bin/env bash
set -e

source /opt/ros/overlay_ws/install/setup.bash

cd /opt/ros/
python3 -m venv semantic_world-venv --system-site-packages  && source semantic_world-venv/bin/activate && pip install -U pip && pip install -U setuptools && pip install -r /opt/ros/overlay_ws/src/semantic_world/requirements.txt && pip install -e Multiverse-Parser


source /opt/ros/semantic_world-venv/bin/activate

exec "$@"