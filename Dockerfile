FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3 nano git libgl1 python3-pip

RUN pip install opencv-python flask pyyaml

RUN git clone https://github.com/julakshah/desk-on-demand.git && \
    cd desk-on-demand 

RUN usermod -aG video root && newgrp video