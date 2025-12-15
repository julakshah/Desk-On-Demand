#!/usr/bin/sh
docker run -it --rm $(for d in /dev/video*; do printf -- '--device=%s ' "$d"; done) \
    $(for d in /dev/gpiochip*; do printf -- "--device %s:%s " "$d" "$d"; done) \
    -e ROS_DOMAIN_ID=99 --network=host --ipc=host --pid=host \
    -e DISPLAY="$DISPLAY" \
    -e XAUTHORITY=/tmp/.Xauthority \
    -v "$HOME/.Xauthority:/tmp/.Xauthority:ro" \ 
    desk-on-demand bash
