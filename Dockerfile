FROM ghcr.io/sloretz/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3 nano git libgl1 python3-pip ros-humble-tf-transformations

RUN pip install opencv-python flask pyyaml gpiozero lgpio mediapipe

RUN git clone https://github.com/julakshah/desk-on-demand.git && \
    cd desk-on-demand 

RUN usermod -aG video root && newgrp video
